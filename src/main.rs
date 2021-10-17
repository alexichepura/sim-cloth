use bevy::{
    app::App,
    asset::Assets,
    diagnostic::FrameTimeDiagnosticsPlugin,
    ecs::system::{Commands, ResMut},
    math::Vec3,
    pbr::{prelude::StandardMaterial, LightBundle, PbrBundle},
    prelude::{CoreStage, IntoSystem, Msaa, PerspectiveCameraBundle, Query, With},
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
        BallJoint, ColliderBundle, ColliderPositionSync, ColliderShape, Isometry, NoUserData,
        Point, RapierPhysicsPlugin, Real, RigidBodyBundle, RigidBodyPosition, RigidBodyType,
    },
    render::RapierRenderPlugin,
};
use nalgebra::{point, vector};

fn main() {
    App::build()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierRenderPlugin)
        .add_startup_system(setup.system())
        .add_system_to_stage(CoreStage::Update, up.system())
        .run();
}

pub struct ClothJoint;
pub struct Cloth;

pub fn up(mut cloth: Query<(&Mesh, &Transform, With<Cloth>)>) {
    let c = cloth.single_mut().ok();
    if let Some(c) = c {
        let (mesh, _tr, _is) = c;
        // mesh.attribute_mut(name);
    }
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    let mut vertices: Vec<[f32; 3]> = vec![];
    let mut indices: Vec<u32> = vec![];
    let normal: [f32; 3] = [0., 0., 0.];
    let mut normals: Vec<[f32; 3]> = vec![];
    let mut uvs: Vec<[f32; 2]> = vec![];

    let num = 20;
    let joint_half_size = 0.02;
    let joint_distance = 0.04;
    let mut body_handles = Vec::new();
    let joint_rotation = Vec3::new(0., 0., 0.);

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;
            let joint_point = vector![fk * joint_distance, 1.8, fi * joint_distance];
            vertices.push(joint_point.into());
            uvs.push([0.0, 0.0]);
            normals.push(normal);
            let joint_isometry: Isometry<Real> =
                Isometry::new(joint_point.into(), joint_rotation.into());

            let ball_entity = commands
                .spawn_bundle(PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Cube {
                        size: joint_half_size * 2.0,
                    })),
                    material: materials.add(Color::rgb(0.1, 0.1, 0.3).into()),
                    ..Default::default()
                })
                .insert_bundle(RigidBodyBundle {
                    position: RigidBodyPosition {
                        position: joint_isometry,
                        ..Default::default()
                    },
                    ..Default::default()
                })
                .insert_bundle(ColliderBundle {
                    shape: ColliderShape::ball(joint_half_size),
                    ..Default::default()
                })
                .insert(Transform::default())
                .insert(ColliderPositionSync::Discrete)
                .insert(ClothJoint)
                .id();

            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint =
                    BallJoint::new(Point::origin(), point![0.0, 0.0, -joint_distance * 2.0]);
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_handle,
                    ball_entity,
                ),));
            }
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = BallJoint::new(Point::origin(), point![-joint_distance, 0.0, 0.0]);
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_handle,
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

    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(mesh),
            material: materials.add(Color::rgba(0.2, 0.4, 0.2, 0.9).into()),
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
                    vector![0.0, 1.0, 0.0].into(),
                    Vec3::new(0., 0., 0.).into(),
                ),
                ..Default::default()
            },
            ..Default::default()
        })
        .insert_bundle(ColliderBundle {
            shape: ColliderShape::cuboid(cube_half, cube_half, cube_half),
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
        transform: Transform::from_translation(Vec3::new(0.0, 5.0, 5.0)),
        ..Default::default()
    });
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_translation(Vec3::new(2.0, 3.0, 4.0))
            .looking_at(Vec3::default(), Vec3::Y),
        ..Default::default()
    });
}
