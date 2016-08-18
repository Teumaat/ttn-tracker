<?php

$payload = pack( "H*", '3B734F0043CC0700' );

// Unpack into longs
$data = unpack( "l*", $payload );

print( bin2hex( base64_decode( $payload ) ) ) . PHP_EOL;
print_r( $data );
