#!/usr/bin/perl -w
use strict;

sub Usage {
    die "Usage: mkproblem.pl npickup [ndump] [ndepot] [xmin ymin xmax ymax]\n"
      . "       ndump = 2 (default)\n"
      . "       ndepot = x (default is calcuated as a f(npickup)\n"
      . "       extents default to -+100\n";
}

my $npickup = shift @ARGV || Usage();
my $ndump   = shift @ARGV || 2;
my $ndepot  = shift @ARGV || -1;
my $xmin    = shift @ARGV || -100;
my $ymin    = shift @ARGV || -100;
my $xmax    = shift @ARGV ||  100;
my $ymax    = shift @ARGV ||  100;

# DEPOT/VEHICLE OPTIONS
my $capacity = 5000;
my $v_open = 0;
my $v_close = 1440;

# DUMP OPTIONS
my $d_open = 0;
my $d_close = 1440;
my $d_service = 30;

# PICKUP OPTIONS
my $demand = 100;
my $p_open = 0;
my $p_close = 1440;
my $p_service = 5;

my $dx = $xmax - $xmin;
my $dy = $ymax - $ymin;

my $nid = 0;

$ndepot = calcNdepot() if $ndepot < 1;

print "#  ndepot: $ndepot\n";
print "#   ndump: $ndump\n";
print "# npickup: $npickup\n";

writeHeader();

for (my $i=0; $i<$ndepot; $i++) {
    my $x = $xmin + rand() * $dx;
    my $y = $ymin + rand() * $dy;
    print "$nid 0 $x $y $capacity $v_open $v_close 0\n";
    $nid++;
}

for (my $i=0; $i<$ndump; $i++) {
    my $x = $xmin + rand() * $dx;
    my $y = $ymin + rand() * $dy;
    print "$nid 1 $x $y 0 $d_open $d_close $d_service\n";
    $nid++;
}

for (my $i=0; $i<$npickup; $i++) {
    my $x = $xmin + rand() * $dx;
    my $y = $ymin + rand() * $dy;
    print "$nid 2 $x $y $demand $p_open $p_close $p_service\n";
    $nid++;
}




sub calcNdepot {
    my $total_demand = $npickup * $demand;
    my $nv = int($total_demand / ($capacity * 0.8)) + 1;
    while (1) {
        my $avgRadius = ($dx>$dx ? $dx : $dy) / sqrt($npickup) / 0.707;
        my $estDuration = $npickup / $nv * ($avgRadius + $p_service) +
            $dx / $ndump / 0.707 + $ndump * $d_service;
        last if $estDuration < $v_close;
        $nv++;
    }
    return $nv;
}

sub writeHeader {
    print <<EOF
#
# trash collection test problem
#
# type: 0:  vehicle
#       1:  dump
#       2:  pickup
#
# For vehicles:
#   demand - is vehicle capcity
#   x,y - vehicle home location
#   tw_open - start time for the vehicle
#   tw_close - time vechile must be home
#   service - NA
#
# For dump facilities:
#   demand - NA
#   x,y - location of the dump
#   tw_open - time dump opens
#   tw_close - time dump closes
#   service - time to service the dump
#
# For pickup locations
#   demand - potential load to add to vehicle
#   x,y - location of pickup
#   tw_open - earliest pickup time
#   tw_close - latest pickup time
#   service - time to service the pickup
#
# Times are in minutes from 0 hour (mid-night)
#
#nid    type    x       y       demand  tw_open tw_close        service
EOF

}

