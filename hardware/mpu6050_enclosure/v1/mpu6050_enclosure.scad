/* MPU6050 / GY-521 rigid zip-tie housing V1. Units: mm.
 * Image dimensions: PCB 20.4 x 15.6; hole pitch 16.
 * Assumptions: PCB thickness 1.6, holes 2.2 from ends and non-header edge,
 * component envelope 3.5, connected header envelope 16 above PCB.
 * Required prints: body_v + lid. flat_body is an ALTERNATIVE body.
 */
part = "assembly";
show_electronics = true;
$fn = 48;
eps = 0.02;

pcb_l = 20.4;
pcb_w = 15.6;
pcb_t = 1.6;
hole_pitch = 16;
hole_edge_y = 2.2;             // ASSUMED, photo does not dimension this offset.
pcb_hole_d = 3.2;             // ASSUMED, M2 screws plus washers are used.
hole_x = [(pcb_l-hole_pitch)/2,(pcb_l+hole_pitch)/2];
hole_y = pcb_w-hole_edge_y;
pcb_centre = [pcb_l/2,pcb_w/2];

base_t = 5;
support_h = 2;
back_z = base_t+support_h;
front_z = back_z+pcb_t;
case_top = front_z+5;
lid_t = 2;
lid_top = case_top+lid_t;
header_clear_h = 16;
hood_top = front_z+header_clear_h+lid_t;
hood_back_y = 6.8;
wall = 1.8;
gap = 0.5;
inner_x0 = -gap;
inner_x1 = pcb_l+gap;
inner_y0 = -gap;
inner_y1 = pcb_w+gap;
outer_x0 = inner_x0-wall;
outer_x1 = inner_x1+wall;
outer_y0 = inner_y0-wall;
outer_y1 = inner_y1+wall;

base_x0 = -11;
base_x1 = pcb_l+11;
base_y0 = -11;
base_y1 = pcb_w+11;
tie_y = [-6.8,pcb_w+6.8];
tie_w = 6;
tie_h = 2.4;
tie_floor = 3.5;
flange_h = 7.6;
v_depth = 3;
v_angle = 150;

cover_x = [-6,pcb_l+6];
cover_y = pcb_w/2;
ear_r = 4.4;
cover_hole_d = 3.5;
m3_nut_af = 5.9;
m3_nut_roof = base_t+3.4;
board_hole_d = 2.4;
m2_nut_af = 4.4;
m2_nut_roof = back_z-1.2;
board_slot_shift = 0.4;

assert(pcb_l>hole_pitch && hole_edge_y>1.8);
assert(tie_floor>v_depth && flange_h>tie_floor+tie_h);
assert(m2_nut_roof<back_z && m3_nut_roof<case_top);
assert(header_clear_h>=10);

module round_box(x0,x1,y0,y1,z0,h,r=1.2) {
    translate([x0,y0,z0])linear_extrude(height=h)
        offset(r=r)translate([r,r])square([x1-x0-2*r,y1-y0-2*r]);
}
module hex_hole(x,y,z,h,af) {
    translate([x,y,z])rotate([0,0,30])cylinder(d=af/cos(30),h=h,$fn=6);
}
module m2_slot(x,z,h,d=board_hole_d) {
    hull()for(y=[hole_y-board_slot_shift,hole_y+board_slot_shift])
        translate([x,y,z])cylinder(d=d,h=h);
}
module m2_well(x) {
    hull()for(y=[hole_y-board_slot_shift,hole_y+board_slot_shift])
        hex_hole(x,y,-eps,m2_nut_roof+eps,m2_nut_af);
}
module cover_ears(z,h) {
    for(x=cover_x)translate([x,cover_y,z])cylinder(r=ear_r,h=h);
}
module cover_holes(z,h) {
    for(x=cover_x)translate([x,cover_y,z])cylinder(d=cover_hole_d,h=h);
}
module underside_v() {
    half=v_depth/tan((180-v_angle)/2);
    // Tube axis is Y. The 150-degree groove runs the full base length in Y.
    translate([pcb_l/2,base_y1+1,0])rotate([90,0,0])
        linear_extrude(height=base_y1-base_y0+2)
            polygon([[-half,-1],[-half,0],[0,v_depth],[half,0],[half,-1]]);
}

module body(v=true) {
    difference() {
        union() {
            round_box(base_x0,base_x1,base_y0,base_y1,0,base_t,2);
            // Raised strap flanges keep the tie channels above the tube envelope.
            for(y=tie_y)
                round_box(base_x0,base_x1,y-4.2,y+4.2,0,flange_h,1.5);
            difference() {
                round_box(outer_x0,outer_x1,outer_y0,outer_y1,base_t,case_top-base_t);
                translate([inner_x0,inner_y0,base_t-eps])
                    cube([inner_x1-inner_x0,inner_y1-inner_y0,case_top+1]);
                // Low front gate also allows bent headers to point outward.
                translate([-0.2,outer_y0-0.5,back_z-0.5])
                    cube([pcb_l+0.4,3.1,case_top-back_z+2]);
            }
            cover_ears(0,case_top);
            // Short, thick necks from cover posts to the case walls.
            translate([cover_x[0],cover_y-1.6,base_t])
                cube([outer_x0-cover_x[0]+0.7,3.2,case_top-base_t]);
            translate([outer_x1-0.7,cover_y-1.6,base_t])
                cube([cover_x[1]-outer_x1+0.7,3.2,case_top-base_t]);
            for(x=hole_x)
                hull()for(y=[hole_y-board_slot_shift,hole_y+board_slot_shift])
                    translate([x,y,base_t-eps])cylinder(r=3.1,h=support_h+eps);
            // Hard backing contacts, away from the assumed header solder tails.
            translate([pcb_l/2,pcb_w/2,base_t-eps])cylinder(r=1.6,h=support_h+eps);
            translate([5,4.3,base_t-eps])cube([pcb_l-10,0.8,support_h+eps]);
        }
        if(v)underside_v();
        for(y=tie_y)
            translate([base_x0-1,y-tie_w/2,tie_floor])
                cube([base_x1-base_x0+2,tie_w,tie_h]);
        // Extra clearance for untrimmed header solder tails.
        translate([-0.2,-0.2,base_t-1.4])cube([pcb_l+0.4,4.0,back_z-base_t+1.6]);
        for(x=hole_x) {m2_slot(x,-eps,back_z+1);m2_well(x);}
        cover_holes(-eps,case_top+2);
        for(x=cover_x)hex_hole(x,cover_y,-eps,m3_nut_roof+eps,m3_nut_af);
    }
}

module lid() {
    difference() {
        union() {
            difference() {
                round_box(outer_x0,outer_x1,outer_y0,outer_y1,case_top,lid_t);
                translate([inner_x0,outer_y0-0.1,case_top-eps])
                    cube([inner_x1-inner_x0,5.0-outer_y0+0.1,lid_t+2*eps]);
            }
            cover_ears(case_top,lid_t);
            translate([cover_x[0],cover_y-3,case_top])
                cube([outer_x0-cover_x[0]+1,6,lid_t]);
            translate([outer_x1-1,cover_y-3,case_top])
                cube([cover_x[1]-outer_x1+1,6,lid_t]);
            // Tall connector guard, with top and front cable access.
            difference() {
                round_box(outer_x0,outer_x1,outer_y0,hood_back_y,case_top,hood_top-case_top);
                translate([inner_x0,inner_y0,case_top-eps])
                    cube([inner_x1-inner_x0,5.0-inner_y0,hood_top-lid_t-case_top+eps]);
                translate([-0.2,outer_y0-0.2,case_top-eps])
                    cube([pcb_l+0.4,3.1,hood_top-lid_t-case_top+eps]);
                // A full-width roof slot tolerates vertical female connectors/wires.
                round_box(0.1,pcb_l-0.1,-0.4,3.6,hood_top-lid_t-0.1,lid_t+0.2,0.6);
                // Back-wall opening provides a tie route around the cable guard.
                translate([pcb_l/2-2.3,4.8,hood_top-4.8])cube([4.6,2.3,2.2]);
            }
        }
        cover_holes(case_top-1,hood_top-case_top+3);
        translate([pcb_l/2,11.0,lid_top-0.45])linear_extrude(height=0.55)
            text("6050",size=3.5,font="Arial:style=Bold",halign="center",valign="center");
        // Arrow indicates VCC end, not a calibrated accelerometer axis label.
        translate([pcb_l-2,11.0,lid_top-0.45])linear_extrude(height=0.55)
            polygon([[1.2,0],[-1,-1],[-1,1]]);
    }
}

module pcb_proxy() {
    difference() {
        translate([0,0,back_z])cube([pcb_l,pcb_w,pcb_t]);
        for(x=hole_x)translate([x,hole_y,back_z-0.1])cylinder(d=pcb_hole_d,h=pcb_t+0.2);
    }
}
module component_proxy() {
    translate([4.0,4.5,front_z])cube([pcb_l-8,7.5,3.5]);
}
module header_proxy() {
    // Bounding boxes, not a claimed exact model of the user's cable assembly.
    translate([0.04,0.1,front_z])cube([20.32,3.0,2.5]);
    translate([0.04,0.1,front_z+2.5])cube([20.32,3.0,11.0]);
    for(i=[0:7])translate([(pcb_l-7*2.54)/2+i*2.54-0.32,1.28,back_z-3])
        cube([0.64,0.64,3.1]);
}
module fasteners_proxy() {
    // PCB: M2x5 with 0.3mm washer. Cover: M3x12 with 0.4mm washer.
    for(x=hole_x) {
        translate([x,hole_y,front_z+0.3-5])cylinder(d=2,h=5);
        translate([x,hole_y,front_z])cylinder(d=5,h=0.3);
        translate([x,hole_y,front_z+0.3])cylinder(d=3.8,h=2);
        hex_hole(x,hole_y,m2_nut_roof-1.6,1.6,4.0);
    }
    for(x=cover_x) {
        translate([x,cover_y,lid_top+0.4-12])cylinder(d=3,h=12);
        translate([x,cover_y,lid_top])cylinder(d=7,h=0.4);
        translate([x,cover_y,lid_top+0.4])cylinder(d=5.5,h=3);
        hex_hole(x,cover_y,m3_nut_roof-2.4,2.4,5.5);
    }
}

module lid_print() {translate([0,0,hood_top])rotate([180,0,0])lid();}
module print_plate() {translate([12,12,0])body();translate([59,15,0])lid_print();}
module assembly() {
    color("#8752ac")body();color("#bd9bd4")lid();
    if(show_electronics) {
        color("#258d9b")pcb_proxy();color("#29242e")component_proxy();
        color("#e8e3df")header_proxy();color("#c5bdc9")fasteners_proxy();
    }
}

if(part=="body_v")body();
else if(part=="flat_body")body(false);
else if(part=="lid")lid_print();
else if(part=="plate")print_plate();
else if(part=="pcb_gauge")translate([0,0,-back_z])pcb_proxy();
else if(part=="assembly")assembly();
else if(part=="check_body_lid")intersection(){body();lid();}
else if(part=="check_body_pcb")intersection(){body();pcb_proxy();}
else if(part=="check_lid_pcb")intersection(){lid();pcb_proxy();}
else if(part=="check_body_header")intersection(){body();header_proxy();}
else if(part=="check_lid_header")intersection(){lid();header_proxy();}
else if(part=="check_lid_components")intersection(){lid();component_proxy();}
else if(part=="check_body_fasteners")intersection(){body();fasteners_proxy();}
else if(part=="check_lid_fasteners")intersection(){lid();fasteners_proxy();}
else if(part=="check_pcb_fasteners")intersection(){pcb_proxy();fasteners_proxy();}
else assert(false,"Unknown part");
