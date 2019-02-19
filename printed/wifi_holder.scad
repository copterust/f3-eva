od = 6.4;   // outer handle diameter
id = 3.6;   // inner diameter
ol = 26.4;  // distance between outsides
sh = 4.5;   // available screw length

module stands_lower() {
    difference () {
        union () {
            translate([ol / 2 - od / 2, 0, 0])
                cylinder(r1 = od / 2, r2 = od / 2, h = sh / 2);
            translate([-ol / 2 + od / 2, 0, 0])
                cylinder(r1 = od / 2, r2 = od / 2, h = sh / 2);
            
            translate([-9, 0, 0]) cube([18, 3, sh / 2]);
        }
        translate([ol / 2 - od / 2, 0, -0.5])
            cylinder(r1 = id / 2, r2 = id / 2, h = sh + 1);
        translate([-ol / 2 + od / 2, 0, -0.5])
            cylinder(r1 = id / 2, r2 = id / 2, h = sh + 1);
    }
}

module lower() {
    difference() {
        stands_lower();
        translate([-15.5 / 2, -1.1, 1]) cube([15.5, 1.1, 5 + 1]);
        translate([-10.5 / 2, -1, 1]) cube([10.5, 5, 5]);
    }
}

module stands_upper() {
    translate([0, 0, sh / 2]) difference () {
        union () {
            translate([ol / 2 - od / 2, 0, 0])
                cylinder(r1 = od / 2, r2 = od / 2, h = sh / 2);
            translate([-ol / 2 + od / 2, 0, 0])
                cylinder(r1 = od / 2, r2 = od / 2, h = sh / 2);
            
            translate([-9, 0, 0]) cube([18, 3, sh + 0.25]);
        }
        translate([ol / 2 - od / 2, 0, -0.5])
            cylinder(r1 = id / 2, r2 = id / 2, h = sh + 1);
        translate([-ol / 2 + od / 2, 0, -0.5])
            cylinder(r1 = id / 2, r2 = id / 2, h = sh + 1);
    }
}

module upper() {
    difference() {
        stands_upper();
        translate([-15.5 / 2, -1.1, 1]) cube([15.5, 1.1, 5 + 1]);
        translate([-10.5 / 2, -1, 1]) cube([10.5, 5, 5]);
        translate([0, 0, sh ]) union() {
            translate([ol / 2 - od / 2, 0, 0])
                cylinder(r1 = od / 2, r2 = od / 2, h = sh / 2 + 1);
            translate([-ol / 2 + od / 2, 0, 0])
                cylinder(r1 = od / 2, r2 = od / 2, h = sh / 2 + 1);
        }
    }
}

$fn = 200;

color("red", 1.0) lower();
color("green", 1.0) upper();