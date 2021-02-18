pcb_width = 107;
pcb_height = 77;
pcb_thickness = 1;
edge_cutout_radius = 6;
mount_hole_radius = 1.5;
mount_hole_distance_x = 78;
mount_hole_distance_y = 51;
bh1750_mount_hole_distance = 8.7;
bh1750_height = 20;
ina219_mount_hole_distance_x = 20;
ina219_mount_hole_distance_y = 17;

difference()
{
    cube([pcb_width, pcb_height, pcb_thickness]);  
    translate([0,0,-.5])
    {
        edge_cutout();
    }
    translate([0,pcb_height,-.5])
    {
        edge_cutout();
    }
    translate([pcb_width,0,-.5])
    {
        edge_cutout();
    }
    translate([pcb_width,pcb_height,-.5])
    {
        edge_cutout();
    }

    // The four holes to mount the PCB to the case

    translate([(pcb_width - mount_hole_distance_x)/2, (pcb_height - mount_hole_distance_y)/2,-.5])
    {
        mount_hole();
    }

    translate([pcb_width-(pcb_width - mount_hole_distance_x)/2, (pcb_height - mount_hole_distance_y)/2,-.5])
    {
        mount_hole();
    }
    
    translate([(pcb_width - mount_hole_distance_x)/2, pcb_height - (pcb_height - mount_hole_distance_y)/2,-.5])
    {
        mount_hole();
    }

    translate([pcb_width-(pcb_width - mount_hole_distance_x)/2, pcb_height - (pcb_height - mount_hole_distance_y)/2,-.5])
    {
        mount_hole();
    }    
    
    // The two mount holes for the BH1750 light sensor
    
    translate([(pcb_width - mount_hole_distance_x)/2 - bh1750_mount_hole_distance, (pcb_height - mount_hole_distance_y)/2 +  10,-.5])
    {
        mount_hole();
    }
    
    translate([(pcb_width - mount_hole_distance_x)/2, (pcb_height - mount_hole_distance_y)/2 + 10,-.5])
    {
        mount_hole();
    }
    
    // The single mount hole for the BME208
    
    translate([(pcb_width - mount_hole_distance_x)/2 - bh1750_mount_hole_distance, (pcb_height - mount_hole_distance_y)/2 +  10 + bh1750_height + 5,-.5])
    {
        mount_hole();
    }

    // Four holes for the INA219 Current Sensor
    
    translate([pcb_width/2 + 5, 5,-.5])
    {
        mount_hole();
    }

    translate([pcb_width/2 + 5 , 5,-.5])
    {
        mount_hole();
    }

    
    
}


module edge_cutout()
{
    cylinder(pcb_thickness + 1, edge_cutout_radius,  edge_cutout_radius, $fn=1000);   
}

module mount_hole()
{
    cylinder(pcb_thickness + 1, mount_hole_radius, mount_hole_radius, $fn=1000);
}
