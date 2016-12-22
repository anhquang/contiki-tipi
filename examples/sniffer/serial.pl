#!/usr/bin/perl

use strict;
use warnings;

use Device::SerialPort;

my $port1_path = '/dev/ttyUSB0';
my $port2_path = '/dev/ttyUSB1';

my $port1 = Device::SerialPort->new($port1_path);
$port1->databits(8);
$port1->baudrate(19200);
$port1->parity("none");
$port1->stopbits(1);

my $port2 = Device::SerialPort->new($port2_path);
$port2->databits(8);
$port2->baudrate(19200);
$port2->parity("none");
$port2->stopbits(1);

while ($in = $port1->input) {
    $port2->write($in);
}
