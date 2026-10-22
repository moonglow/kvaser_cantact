/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * http://www.wtfpl.net/ for more details. */

/*
 * Line-rate CAN traffic source through Kvaser CANlib (linuxcan), used by
 * tools/validate_bench.py to saturate a bus that the Linux kvaser_usb
 * SocketCAN driver can only fill to about 3,000 frames/s from a Leaf.
 *
 * Sends <count> 8-byte frames with a fixed standard ID and a little-endian
 * 64-bit sequence counter (0, 1, 2, ...) as payload, the same format as
 * `cangen -D i -L 8`, so the same loss oracle applies.
 *
 * With <rx-id> <rx-count>, it also checks what it receives while sending:
 * frames with <rx-id> must carry the sequence 0..rx-count-1 in order, with
 * nothing missing (the other direction of a bidirectional full-load test).
 *
 *   canlib_sender <channel> <bitrate> <count> <id-hex> [<rx-id-hex> <rx-count>]
 *
 * Prints one result line of key=value pairs; times are CLOCK_MONOTONIC
 * seconds, the same clock as Python's time.monotonic().
 *
 * Build: gcc -O2 -o canlib_sender canlib_sender.c -lcanlib
 */
#include <canlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static long bitrate_const( long bitrate )
{
  switch( bitrate )
  {
    case 1000000: return canBITRATE_1M;
    case 500000:  return canBITRATE_500K;
    case 250000:  return canBITRATE_250K;
    case 125000:  return canBITRATE_125K;
    default:      return 0;
  }
}

static void check( const char *what, canStatus st )
{
  if( st != canOK )
  {
    char msg[64] = "";
    canGetErrorText( st, msg, sizeof( msg ) );
    fprintf( stderr, "%s: %s (%d)\n", what, msg, (int)st );
    exit( 1 );
  }
}

static double now( void )
{
  struct timespec t;
  clock_gettime( CLOCK_MONOTONIC, &t );
  return t.tv_sec + t.tv_nsec / 1e9;
}

/* receive-side state for the optional sequence check */
static long rx_id = -1;
static uint64_t rx_expected = 0, rx_count = 0;
static uint64_t rx_next = 0;      /* sequence number expected next */
static uint64_t rx_missing = 0;   /* sequence numbers skipped over */
static uint64_t rx_disorder = 0;  /* duplicates or going backwards */
static uint64_t rx_overruns = 0;  /* frames flagged as overrun by this Kvaser itself */
static uint64_t rx_errframes = 0;
static double rx_first = 0, rx_last = 0;

static void drain_rx( canHandle h )
{
  long id;
  uint8_t data[8];
  unsigned int dlc, flags;
  unsigned long ts;

  while( canRead( h, &id, data, &dlc, &flags, &ts ) == canOK )
  {
    if( flags & ( canMSGERR_OVERRUN | canMSGERR_HW_OVERRUN | canMSGERR_SW_OVERRUN ) )
      rx_overruns++;
    if( flags & canMSG_ERROR_FRAME )
    {
      rx_errframes++;
      continue;
    }
    if( id != rx_id || ( flags & canMSG_TXACK ) || dlc != 8 )
      continue;
    uint64_t seq = 0;
    for( int b = 0; b < 8; b++ )
      seq |= (uint64_t)data[b] << ( 8 * b );
    if( seq > rx_next )
      rx_missing += seq - rx_next;
    if( seq < rx_next )
      rx_disorder++;
    else
      rx_next = seq + 1;
    if( !rx_count )
      rx_first = now();
    rx_last = now();
    rx_count++;
  }
}

int main( int argc, char **argv )
{
  if( argc != 5 && argc != 7 )
  {
    fprintf( stderr, "usage: %s <channel> <bitrate> <count> <id-hex> [<rx-id-hex> <rx-count>]\n", argv[0] );
    return 2;
  }
  int channel = atoi( argv[1] );
  long bitrate = bitrate_const( atol( argv[2] ) );
  uint64_t count = strtoull( argv[3], NULL, 10 );
  long id = strtol( argv[4], NULL, 16 );
  if( argc == 7 )
  {
    rx_id = strtol( argv[5], NULL, 16 );
    rx_expected = strtoull( argv[6], NULL, 10 );
  }
  if( !bitrate )
  {
    fprintf( stderr, "unsupported bitrate %s\n", argv[2] );
    return 2;
  }

  canInitializeLibrary();
  canHandle h = canOpenChannel( channel, canOPEN_EXCLUSIVE );
  if( h < 0 )
    check( "canOpenChannel", (canStatus)h );
  check( "canSetBusParams", canSetBusParams( h, bitrate, 0, 0, 0, 0, 0 ) );
  check( "canSetBusOutputControl", canSetBusOutputControl( h, canDRIVER_NORMAL ) );
  check( "canBusOn", canBusOn( h ) );

  double t0 = now();
  for( uint64_t i = 0; i < count; )
  {
    uint8_t data[8];
    for( int b = 0; b < 8; b++ )
      data[b] = (uint8_t)( i >> ( 8 * b ) );
    canStatus st = canWrite( h, id, data, 8, canMSG_STD );
    if( st == canERR_TXBUFOFL )
    {
      /* transmit queue full: retry shortly. canWriteSync would wait for the
         whole queue to empty and leave the bus idle while it refills. */
      if( rx_id >= 0 )
        drain_rx( h );
      struct timespec pause = { 0, 50000 };
      nanosleep( &pause, NULL );
      continue;
    }
    check( "canWrite", st );
    i++;
    /* keep the receive queue empty so the check measures the bus, not us */
    if( rx_id >= 0 )
      drain_rx( h );
  }
  if( count )
    check( "canWriteSync", canWriteSync( h, 10000 ) );
  double t1 = now();

  /* keep receiving until the other sender has gone quiet for a second */
  if( rx_id >= 0 )
  {
    double quiet_from = now();
    uint64_t seen = rx_count;
    /* receive-only runs wait longer for the other side to start */
    double quiet = count ? 1.0 : 3.0;
    while( now() - quiet_from < quiet && rx_count < rx_expected )
    {
      drain_rx( h );
      if( rx_count != seen )
      {
        seen = rx_count;
        quiet_from = now();
      }
      struct timespec pause = { 0, 200000 };
      nanosleep( &pause, NULL );
    }
    drain_rx( h );
  }

  printf( "tx=%llu tx_start=%.6f tx_end=%.6f tx_rate=%.0f",
          (unsigned long long)count, t0, t1, count ? count / ( t1 - t0 ) : 0.0 );
  if( rx_id >= 0 )
  {
    /* frames missing at the very end are not followed by a later sequence */
    if( rx_next < rx_expected )
      rx_missing += rx_expected - rx_next;
    printf( " rx=%llu rx_expected=%llu rx_missing=%llu rx_disorder=%llu rx_overruns=%llu"
            " rx_errframes=%llu rx_first=%.6f rx_last=%.6f",
            (unsigned long long)rx_count, (unsigned long long)rx_expected,
            (unsigned long long)rx_missing, (unsigned long long)rx_disorder,
            (unsigned long long)rx_overruns, (unsigned long long)rx_errframes, rx_first, rx_last );
  }
  printf( "\n" );

  canBusOff( h );
  canClose( h );
  return ( rx_id >= 0 && ( rx_count != rx_expected || rx_missing || rx_disorder ) ) ? 3 : 0;
}
