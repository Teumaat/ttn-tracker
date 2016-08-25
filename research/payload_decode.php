<?php

## 32 bit ints
//$payload = pack( "H*", '3B734F0043CC0700' );
// Unpack into longs
//$data = unpack( "l*", $payload );
//print( bin2hex( base64_decode( $payload ) ) ) . PHP_EOL;
//print_r( $data );

## 24 bit ints
$payload = pack( "H*", 'EBF107A0C700' );

// Unpack into 6 bytes
$data = unpack('C*', $payload);

// Combine each 3 bytes into a 24bit int
$lat = ($data[1] + ($data[2] << 8) + ($data[3] << 16)) / 10000;
$lon = ($data[4] + ($data[5] << 8) + ($data[6] << 16)) / 10000;
var_dump( $lat, $lon );
