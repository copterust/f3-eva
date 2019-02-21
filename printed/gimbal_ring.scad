// Quadcopter test gimbal for the https://github.com/copterust/ project

// Based on 105 frame
dfh = 88;   // height of frame by the outermost side of motors
dfw = 90;   // width by the outermost points
dft = 4;    // thickness of a bottom frame
mhd = 12.5; // motor handle diameter
mhh = 8;    // motor handle height from the bottom of the frame
md = 8.5;   // motor diameter
mh = 50;    // height of the motor from the bottom of the frame
pd = 60;    // propeller diameter
pt = 5;     // propeller thickness
margin = 3; // margin between ring and tip of the propeller

// Based on 608 type bearing
bhw = 3;    // bearing house width
bh = 7;     // bearing height
bod = 23;   // bearing outer diameter (with tolerance)
bsd = 11;   // bearing shaft diameter (with tolerance)
bid = 7.9;  // bearing inner (hole) diameter (with tolerance)

// Lenght of a peg to the next ring
pl = 17;

// Number of screw holes (double check if changing this value)
hn = 6;

// Ring height (will be no higher that bearing)
rh = 4;

$fn = 300;

// Mock for a drone
module drone() {
    union() {
        // Base plate for a drone
        cube([dfw, dfh, dft], true);

        // Motor handlers
        mhr = mhd / 2;
        for (i = [0:1:4]) {
            sy = sign(sin(45 + i * (360 / 4)));
            sx = sign(cos(45 + i * (360 / 4)));
            x = sx * (dfw / 2 - mhr);
            y = sy * (dfh / 2 - mhr);
            translate([x, y, mhh / 2 - dft / 2])
                cylinder(r1 = mhr, r2 = mhr, mhh, center = true);
        }

        // Motors
        mr = md / 2;
        for (i = [0:1:4]) {
            sy = sign(sin(45 + i * (360 / 4)));
            sx = sign(cos(45 + i * (360 / 4)));
            x = sx * (dfw / 2 - mhr);
            y = sy * (dfh / 2 - mhr);
            translate([x, y, mh / 2 - dft / 2])
                cylinder(r1 = mr, r2 = mr, mh, center = true);
        }
        // Propellers
        pr = pd / 2;
        for (i = [0:1:4]) {
            sy = sign(sin(45 + i * (360 / 4)));
            sx = sign(cos(45 + i * (360 / 4)));
            x = sx * (dfw / 2 - mhr);
            y = sy * (dfh / 2 - mhr);
            translate([x, y, mh - dft / 2])
                cylinder(r1 = pr, r2 = pr, pt, center = true);
        }
    }
}

module ring(ird) {
    union() {
        difference () {
            cylinder(r1 = ird / 2 + bh,
                     r2 = ird / 2 + bh,
                     h = (bod / 2) + bhw / 2);
            translate([0, 0, -1])
                cylinder(r1 = ird / 2,
                         r2 = ird / 2,
                         h = bod / 2 + 2 + bhw);
        }
        translate([ird / 2, 0, 0])
            rotate([0, 90, 0])
                cylinder(r1 = bod / 2 + bhw / 2,
                         r2 = bod / 2 + bhw / 2,
                         h = bhw + bh);
        translate([-ird / 2 - bh - bhw, 0, 0])
            rotate([0, 90, 0])
                cylinder(r1 = bod / 2 + bhw / 2,
                         r2 = bod / 2 + bhw / 2,
                         h = bhw + bh);
    }
}

module half_ring(ird, pegs = true) {
    difference() {
        union() {
            ring(ird);
                // pegs to next ring
                if (pegs) {
                translate([0, bhw * 2 + ird / 2 + pl / 2, 0])
                    rotate([90, 0, 0]) 
                        cylinder(r1 = bid / 2,
                                 r2 = bid / 2,
                                 h = pl,
                                 center = true);
                translate([0, -(bhw * 2 + ird / 2 + pl / 2), 0])
                    rotate([90, 0, 0]) 
                        cylinder(r1 = bid / 2,
                                 r2 = bid / 2,
                                 h = pl,
                                 center = true);
            }
        }
        translate([ird / 2 + bh / 2 + bhw / 2, 0, 0])
            rotate([0, 90, 0])
                cylinder(r1 = bod / 2,
                         r2 = bod / 2,
                         h = bh,
                         center = true);
        translate([-ird / 2 - bh / 2 - bhw / 2, 0, 0])
            rotate([0, 90, 0])
                cylinder(r1 = bod / 2,
                         r2 = bod / 2,
                         h = bh,
                         center = true);
        translate([ird / 2 + bh / 2 - 2 + bhw, 0, 0])
            rotate([0, 90, 0])
                cylinder(r1 = bsd / 2 + bhw / 2,
                         r2 = bsd / 2 + bhw / 2,
                         h = bh + 4 + bhw * 2,
                         center = true);
        translate([-ird / 2 - bh / 2 + 2 - bhw, 0, 0])
            rotate([0, 90, 0])
                cylinder(r1 = bsd / 2 + bhw / 2,
                         r2 = bsd / 2 + bhw / 2,
                         h = bh + 4 + bhw * 2,
                         center = true);
        translate([0, 0, -50])
            cube([ird * 4, ird * 4, 100], center = true);

        for (i = [0: 1: hn]) {
            rotate([0, 0, (360 / hn) / 2 + i * 360 / hn]) 
                translate([ird / 2 + bh / 2, 0, bod / 2])
                    cylinder(r1 = sd / 2,
                             r2 = sd / 2,
                             h = bod,
                             center = true);
        }

        difference() {
            translate([0, 0, 50 + rh])
                cube([ird * 4, ird * 4, 100], center = true);
            translate([ird / 2, 0, 0]) rotate([0, 90, 0])
                cylinder(r1 = bod / 2 + bhw / 2,
                         r2 = bod / 2 + bhw / 2,
                         h = bhw * 2 + bh);
            translate([-ird / 2 - bh - bhw, 0, 0])
                rotate([0, 90, 0])
                    cylinder(r1 = bod / 2 + bhw / 2,
                             r2 = bod / 2 + bhw / 2,
                             h = bhw + bh);
        }
    }
}

// Difference between corner and motor handle
d = sqrt(2 * pow(mhd / 2, 2));
// Max radius at the bottom
rb = sqrt(pow((dfh / 2), 2) + pow((dfw / 2), 2)) + pd / 2 - d;
// Max radius at the propeller tip
rp = sqrt(pow(rb, 2) + pow(mh - dft / 2 + pt / 2, 2));
// Inner ring diameter
ird = 2 * (rp + margin);

// Don't forget to comment this out before exporting STL!
rotate([36.9, 0, 0]) color("blue", 0.5) drone();

// Printed parts

// Ring 0
half_ring(ird);

// Ring 1
h = (2 * (bhw + bh) + ird) / 2;
ird2 = sqrt(pow(h, 2) + pow(bod / 2 + bhw / 2, 2));
rotate([0, 0, 90])
    half_ring((ird2 + margin) * 2);

// Ring 2
h2 = (2 * (bhw + bh) + (ird2 + margin) * 2) / 2;
ird3 = sqrt(pow(h2, 2) + pow(bod / 2 + bhw / 2, 2));
half_ring((ird3 + margin) * 2);

// Outer ring
h3 = (2 * (bhw + bh) + (ird3 + margin) * 2) / 2;
ird4 = sqrt(pow(h3, 2) + pow(bod / 2 + bhw / 2, 2));
rotate([0, 0, 90])
    half_ring((ird4 + margin) * 2, pegs = false);
