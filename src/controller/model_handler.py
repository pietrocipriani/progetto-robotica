MODEL_SDF = """<?xml version="1.0" ?>
<sdf version="1.4">
<model name="brick_%s">
<link name="link">
	<inertial>
		<mass>1</mass>
		<inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
		</inertia>
	</inertial>

	<collision name="collision">
        <laser_retro>0</laser_retro>
        <max_contacts>10</max_contacts>
		<geometry>
			<mesh>
			    <uri>model://brick_%s/mesh.stl</uri>
			</mesh>
		</geometry>
        <surface>
            <friction>
                <ode>
                    <mu>1</mu>
                    <mu2>1</mu2>
                    <fdir1>0 0 0</fdir1>
                    <slip1>0</slip1>
                    <slip2>0</slip2>
                </ode>
                <torsional>
                    <coefficient>1</coefficient>
                    <patch_radius>0</patch_radius>
                    <surface_radius>0</surface_radius>
                    <use_patch_radius>1</use_patch_radius>
                    <ode>
                        <slip>0</slip>
                    </ode>
                </torsional>
            </friction>
            <bounce>
                <restitution_coefficient>0</restitution_coefficient>
                <threshold>1e+06</threshold>
            </bounce>
            <contact>
                <collide_without_contact>0</collide_without_contact>
                <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
                <collide_bitmask>1</collide_bitmask>
                <ode>
                    <soft_cfm>0</soft_cfm>
                    <soft_erp>0.2</soft_erp>
                    <kp>1e+13</kp>
                    <kd>1</kd>
                    <max_vel>0.01</max_vel>
                    <min_depth>0</min_depth>
                </ode>
                <bullet>
                    <split_impulse>1</split_impulse>
                    <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                    <soft_cfm>0</soft_cfm>
                    <soft_erp>0.2</soft_erp>
                    <kp>1e+13</kp>
                    <kd>1</kd>
                </bullet>
            </contact>
        </surface>
	</collision>

	<visual name="visual">
		<geometry>
			<mesh>
			<uri>model://brick_%s/mesh.stl</uri>
			</mesh>
		</geometry>
		<material>
			<ambient>1.0 1.0 0.7 1.0</ambient>
			<diffuse>1.0 1.0 0.7 1.0</diffuse>
			<specular>0.01 0.01 0.01 1 1.5</specular>
			<emissive>0.0 0.0 0.0 0.0</emissive>
		</material>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
	</visual>
</link>
</model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0" ?>
<model>
    <name>brick_%s</name>
    <version>1.0</version>
    <sdf version="1.7">model.sdf</sdf>
    <author>
        <name></name>
        <email></email>
    </author>
    <description></description>
</model>
"""

NAMES = ["1x1_H", "2x1_T", "2x1_L", "2x1_H", "2x1_U", "2x2_H", "2x2_U", "3x1_H", "3x1_U", "4x1_H", "4x1_L"]

for name in NAMES:
    with open(f"models/brick_{name}/model.config", "w") as f:
        f.write(MODEL_CONFIG % name)
    with open(f"models/brick_{name}/model.sdf", "w") as f:
        f.write(MODEL_SDF % (name, name, name))
