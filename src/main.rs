use bevy::{
    app::App,
    asset::Assets,
    diagnostic::FrameTimeDiagnosticsPlugin,
    ecs::system::{Commands, ResMut},
    math::Vec3,
    pbr::{prelude::StandardMaterial, LightBundle, PbrBundle},
    prelude::{
        AssetServer, CoreStage, Handle, IntoSystem, Msaa, PerspectiveCameraBundle, Query, QuerySet,
        Res,
    },
    render::{
        color::Color,
        mesh::Mesh,
        mesh::{shape, Indices, VertexAttributeValues},
        pipeline::PrimitiveTopology,
    },
    transform::components::Transform,
    DefaultPlugins,
};
use bevy_rapier3d::{
    physics::JointBuilderComponent,
    prelude::{
        BallJoint, CoefficientCombineRule, ColliderBundle, ColliderMaterial, ColliderPositionSync,
        ColliderShape, Isometry, MassProperties, NoUserData, Point, RapierPhysicsPlugin, Real,
        RigidBodyBundle, RigidBodyDamping, RigidBodyPosition, RigidBodyType, SpringModel, Vector,
    },
    render::RapierRenderPlugin,
};
use nalgebra::{point, vector};

fn main() {
    App::build()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .init_resource::<State>()
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierRenderPlugin)
        .add_system_to_stage(CoreStage::PreUpdate, up.system())
        .add_startup_system(setup.system())
        .run();
}

pub struct ClothJoint;
pub struct Cloth;

#[derive(Default)]
struct State {
    handle: Handle<Mesh>,
}

fn up(
    mut cloth_set: QuerySet<(
        Query<(&Transform, &ClothJoint)>,
        Query<(&mut Transform, &Cloth)>,
    )>,
    state: Res<State>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let mut vertices: Vec<[f32; 3]> = vec![];
    let mut normals: Vec<[f32; 3]> = vec![];
    cloth_set.q0().for_each(|joint_body| {
        let rot = joint_body.0.rotation;
        let normal_shift = rot.mul_vec3(Vec3::new(0., 0.020, 0.));
        let normal = normal_shift.normalize();
        normals.push(normal.into());

        if vertices.len() == 0 {
            vertices.push([0., 0., 0.]);
        } else {
            let first: [f32; 3] = vertices[0];
            let tr = joint_body.0.translation;
            vertices.push([
                tr.x - first[0] + normal_shift.x,
                tr.y - first[1] + normal_shift.y,
                tr.z - first[2] + normal_shift.z,
            ]);
        }
    });
    let first_vertex: [f32; 3] = vertices[0].clone();

    for (mut transform, _is) in cloth_set.q1_mut().iter_mut() {
        let mesh = meshes.get_mut(state.handle.clone());
        if let Some(mesh) = mesh {
            // https://github.com/bevyengine/bevy/pull/1164
            // let pos = sub.attribute_mut(Mesh::ATTRIBUTE_POSITION).unwrap();
            // if let VertexAttributeValues::Float3(ref mut pos) = pos {
            //     pos.clear();
            //     pos.extend(
            //         triangles
            //             .vertices
            //             .iter()
            //             .map(|v| [v.pos.x, v.pos.y, 0.0f32]),
            //     );
            // }
            mesh.set_attribute(
                Mesh::ATTRIBUTE_POSITION,
                VertexAttributeValues::from(vertices.clone()),
            );
            mesh.set_attribute(
                Mesh::ATTRIBUTE_NORMAL,
                VertexAttributeValues::from(normals.clone()),
            );
        }
        transform.translation = first_vertex.into();
    }
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut state: ResMut<State>,
    asset_server: Res<AssetServer>,
) {
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    let mut vertices: Vec<[f32; 3]> = vec![];
    let mut indices: Vec<u32> = vec![];
    let normal: [f32; 3] = [0., 0., 0.];
    let mut normals: Vec<[f32; 3]> = vec![];
    let mut uvs: Vec<[f32; 2]> = vec![];

    let num = 90;
    let thikness_half = 0.003;
    let joint_half_size = 0.004;
    let joint_half_distance = joint_half_size + 0.001;
    let joint_distance = joint_half_distance * 2.;
    let mut body_handles = Vec::new();
    let joint_init_rot = Vec3::new(0., 0., 0.);
    let joint_init_pos = Vec3::new(0., 1.8, 0.);

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;
            let joint_vertice = vector![fk * joint_distance, 0., fi * joint_distance];
            vertices.push(joint_vertice.into());
            uvs.push([fk / 100.0, fi / 100.0]);
            normals.push(normal);

            let joint_point = vector![
                joint_init_pos.x + joint_vertice[0],
                joint_init_pos.y + joint_vertice[1],
                joint_init_pos.z + joint_vertice[2]
            ];
            let joint_isometry: Isometry<Real> =
                Isometry::new(joint_point.into(), joint_init_rot.into());

            let ball_entity = commands
                // .spawn_bundle(PbrBundle {
                //     mesh: meshes.add(Mesh::from(shape::Box {
                //         max_x: joint_half_size,
                //         min_x: -joint_half_size,
                //         max_y: thikness_half,
                //         min_y: -thikness_half,
                //         max_z: joint_half_size,
                //         min_z: -joint_half_size,
                //     })),
                //     material: materials.add(Color::rgb(0.1, 0.1, 0.3).into()),
                //     ..Default::default()
                // })
                .spawn_bundle(RigidBodyBundle {
                    position: RigidBodyPosition {
                        position: joint_isometry,
                        ..Default::default()
                    },
                    damping: RigidBodyDamping {
                        linear_damping: 1.0,
                        // angular_damping: 1000.0,
                        angular_damping: 1000000000000.0,
                        // angular_damping: 10000000.0,
                        ..Default::default()
                    },

                    ..Default::default()
                })
                .insert_bundle(ColliderBundle {
                    // shape: ColliderShape::ball(joint_half_size),
                    shape: ColliderShape::cuboid(joint_half_size, thikness_half, joint_half_size),
                    mass_properties: MassProperties::new(
                        Vec3::new(0.0, 0.0, 0.0).into(),
                        0.0001,
                        Vec3::new(0.000001, 0.000001, 0.000001).into(),
                        // Vec3::new(1.0, 1.0, 1.0).into(),
                    )
                    .into(),
                    material: ColliderMaterial {
                        friction: 1000.0,
                        restitution: 0.0000001,
                        friction_combine_rule: CoefficientCombineRule::Min,
                        restitution_combine_rule: CoefficientCombineRule::Min,
                        ..Default::default()
                    },
                    ..Default::default()
                })
                .insert(Transform::default())
                .insert(ColliderPositionSync::Discrete)
                .insert(ClothJoint)
                .id();

            let stiffness = 0.0000000000001;
            // let damping = 0.0000000001;
            // let stiffness = 0.2;
            let damping = 10000000000.0;
            // let damping = 1000.0;

            if i > 0 {
                let parent_entity = *body_handles.last().unwrap();
                let mut joint = BallJoint::new(
                    point![0.0, 0.0, joint_half_distance],
                    point![0.0, 0.0, -joint_half_distance],
                );
                // joint.limits_enabled = true;
                // joint.limits_local_axis1 = Vector::y_axis();
                // joint.limits_local_axis2 = Vector::y_axis();
                // joint.limits_angle = 0.0000001;
                // joint.limits_impulse = 0.000001;
                // joint.motor_model = SpringModel::ForceBased;
                joint.motor_stiffness = stiffness;
                joint.motor_damping = damping;
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    ball_entity,
                ),));
            }
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_entity = body_handles[parent_index];
                let mut joint = BallJoint::new(
                    point![joint_half_distance, 0.0, 0.0],
                    point![-joint_half_distance, 0.0, 0.0],
                );
                // joint.limits_enabled = true;
                // joint.limits_local_axis1 = Vector::y_axis();
                // joint.limits_local_axis2 = Vector::y_axis();
                // joint.limits_angle = 0.0000001;
                // joint.limits_impulse = 0.000001;
                // joint.motor_model = SpringModel::ForceBased;
                joint.motor_stiffness = stiffness;
                joint.motor_damping = damping;
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    ball_entity,
                ),));
            }
            body_handles.push(ball_entity);
        }
    }

    let unum = num as u32;
    for row in 0..num {
        for col in 0..num {
            let urow = row as u32;
            let ucol = col as u32;
            // A--B
            // |\ |   This cube face has two triangles:
            // | \|   ABD and ADC.
            // C--D
            // 0, 1, 3, 0, 3, 2
            let p_a: u32 = urow * unum + ucol;
            let p_b: u32 = p_a + 1;
            let p_c: u32 = p_a + unum;
            let p_d: u32 = p_a + 1 + unum;

            indices.push(p_a);
            indices.push(p_b);
            indices.push(p_d);

            indices.push(p_a);
            indices.push(p_d);
            indices.push(p_c);
        }
    }

    mesh.set_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::from(vertices.clone()),
    );
    mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, VertexAttributeValues::from(normals));
    mesh.set_attribute(Mesh::ATTRIBUTE_UV_0, VertexAttributeValues::from(uvs));
    mesh.set_indices(Some(Indices::U32(indices)));

    let mesh_handle = meshes.add(mesh);
    state.handle = mesh_handle.clone();

    let texture_handle = asset_server.load("texture.png");
    commands
        .spawn_bundle(PbrBundle {
            mesh: mesh_handle.clone(),
            material: materials.add(StandardMaterial {
                base_color_texture: Some(texture_handle.clone()),
                // roughness: 0.2,
                ..Default::default()
            }),
            // material: materials.add(Color::rgba(0.4, 0.3, 0.3, 0.95).into()),
            // material: materials.add(Color::rgb(0.4, 0.3, 0.3).into()),
            ..Default::default()
        })
        .insert(Transform::default())
        .insert(Cloth);

    let cube_half = 0.5;
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube {
                size: cube_half * 2.0,
            })),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
            // material: materials.add(Color::rgb(0.1, 0.1, 0.3).into()),
            ..Default::default()
        })
        .insert_bundle(RigidBodyBundle {
            position: RigidBodyPosition {
                position: Isometry::new(
                    vector![0.1, 1.0, 0.1].into(),
                    Vec3::new(0., 0., 0.).into(),
                ),
                ..Default::default()
            },
            ..Default::default()
        })
        .insert_bundle(ColliderBundle {
            shape: ColliderShape::cuboid(cube_half, cube_half, cube_half),
            material: ColliderMaterial {
                friction: 1000.,
                restitution: 0.9,
                ..Default::default()
            },
            ..Default::default()
        })
        .insert(Transform::default())
        .insert(ColliderPositionSync::Discrete)
        .id();

    let plane_half = 10.0;
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Plane {
                size: plane_half * 2.0,
            })),
            material: materials.add(Color::rgb(0.2, 0.6, 0.2).into()),
            ..Default::default()
        })
        .insert_bundle(RigidBodyBundle {
            body_type: RigidBodyType::Static,
            position: RigidBodyPosition {
                position: Isometry::new(
                    vector![0.0, 0.0, 0.0].into(),
                    Vec3::new(0., 0., 0.).into(),
                ),
                ..Default::default()
            },
            ..Default::default()
        })
        .insert_bundle(ColliderBundle {
            shape: ColliderShape::cuboid(plane_half, 0.5, plane_half),
            ..Default::default()
        });
    commands.spawn_bundle(LightBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 6.0, 6.0)),
        ..Default::default()
    });
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_translation(Vec3::new(2.0, 3.0, 3.0))
            .looking_at(Vec3::default(), Vec3::Y),
        ..Default::default()
    });
}
