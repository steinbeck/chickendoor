extrusion_length = 35;
hole_distance = 25;  // the distance of the mounting hole to the wall

mounting_plate_thickness = 5;
mounting_plate_height = 40;
mounting_plate_width = 60;
wall_screw_radius = 2;
m_screw_radius = 4; // The radius of the mounting hole in the actuator
extrusion_width = 5;
gap_width= 20;
wall_thickness = 5;
extrusion_height = (m_screw_radius + wall_thickness)*2;
extrusion_translation = mounting_plate_width /2 - gap_width / 2 - wall_thickness;

    union(){
        translate([0,mounting_plate_height - extrusion_height,extrusion_translation])
        {
            extrusion();
        }
        translate([0,mounting_plate_height - extrusion_height, mounting_plate_width - extrusion_translation - wall_thickness])
        {
            extrusion();
        }

        mounting_plate();
    }
/*
* The part that extrudes from the mounting plate and 
*
*/
module extrusion()
{
       difference(){
            
            union()
            {    
                cube([extrusion_length, (m_screw_radius + wall_thickness)*2, extrusion_width]);    
                translate([extrusion_length,extrusion_height/2,extrusion_width/2])
                {
                    cylinder(extrusion_width, m_screw_radius + wall_thickness, m_screw_radius + wall_thickness, true, $fn=1000);
                }
            }
            translate([extrusion_length, extrusion_height/2, extrusion_width/2 - 1])
            {
                cylinder(extrusion_width * 2, m_screw_radius, m_screw_radius, true, $fn=1000);
            }
        }
        translate([0, -extrusion_height - wall_thickness + 4, extrusion_width])
        {
            rotate([0,90,0])
            {
                prism(extrusion_width, mounting_plate_height - extrusion_height - 2, extrusion_length * 1.1);
            }
        }

}

module mounting_plate()
{
  
difference()
  {  
    cube([mounting_plate_thickness, mounting_plate_height, mounting_plate_width]);  
  
    translate([1, mounting_plate_height/2, mounting_plate_width*.15])
    {
            rotate([0,90,0])
            {
                wall_screw_hole();
            }
     }

    translate([1, mounting_plate_height/2, mounting_plate_width*.85])
    {
            rotate([0,90,0])
            {
                wall_screw_hole();
            }
     }
 }
}


module wall_screw_hole()
{
            cylinder(mounting_plate_thickness * 2 ,wall_screw_radius, wall_screw_radius, true, $fn=1000);
}

module prism(l, w, h){
   polyhedron(
           points=[[0,0,0], [l,0,0], [l,w,0], [0,w,0], [0,w,h], [l,w,h]],
           faces=[[0,1,2,3],[5,4,3,2],[0,4,5,1],[0,3,4],[5,2,1]]
           );
   
}
