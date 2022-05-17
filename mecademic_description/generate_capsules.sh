# Generate cyilinders around meshes of the URDF
robot_capsule_urdf  urdf/meca_500_r3.urdf.xacro --output  urdf/meca_500_r3_capsule.urdf.xacro -c  urdf/capsule
# Generate spheres at the cylinders extremities
robot_capsule_urdf_to_rviz  urdf/meca_500_r3_capsule.urdf.xacro --output  urdf/meca_500_r3_capsule.urdf.xacro
# compute collisions that can be disabled with Moveit for the real URDF
moveit_compute_default_collisions --urdf_path  urdf/meca_500_r3.urdf.xacro --srdf_path  srdf/meca_500_r3.srdf --num_trials 75000000
# compute collisions that can be disabled with Moveit for the capsule URDF
moveit_compute_default_collisions --urdf_path  urdf/meca_500_r3_capsule.urdf.xacro --srdf_path  srdf/meca_500_r3_capsule.srdf --num_trials 75000000
# Now collisions to disable should be check by hand! The ones for the real URDF should be kept, plus adding the "Default" and "Always" collisions from the capsule URDF
YELLOW='\033[1;33m'
printf "${YELLOW}Now collisions to disable should be check by hand! The ones for the real URDF should be kept, plus adding the <Default> and <Always> collisions from the capsule URDF${NC}\n"