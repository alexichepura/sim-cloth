use bevy::{
    app::App,
    asset::Assets,
    diagnostic::FrameTimeDiagnosticsPlugin,
    ecs::system::{Commands, Res, ResMut},
    math::{Quat, Vec3},
    pbr::{prelude::StandardMaterial, PbrBundle},
    prelude::{AssetServer, BuildChildren, IntoSystem, Msaa},
    render::{color::Color, mesh::shape, mesh::Mesh},
    transform::components::Transform,
    DefaultPlugins,
};
use bevy_rapier3d::{
    physics::JointBuilderComponent,
    prelude::{
        BallJoint, ColliderBundle, ColliderMaterial, ColliderPositionSync, ColliderShape, Isometry,
        MassProperties, NoUserData, Point, RapierPhysicsPlugin, Real, RevoluteJoint,
        RigidBodyBundle, RigidBodyPosition, RigidBodyType, SpringModel, Vector,
    },
    render::{ColliderDebugRender, RapierRenderPlugin},
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
        .run();
}

fn setup(mut commands: Commands, mut materials: ResMut<Assets<StandardMaterial>>) {
    let num = 15;
    let rad = 0.4;
    let shift = 1.0;
    let mut body_handles = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;
            // let status = RigidBodyType::Dynamic;
            let t = vector![fk * shift, 0.0, fi * shift * 2.0];
            let qvec = Vec3::new(0., 0., 0.);
            let ball_isometry: Isometry<Real> = Isometry::new(t.into(), qvec.into());

            let ball_entity = commands
                .spawn_bundle(PbrBundle {
                    material: materials.add(Color::rgb(0.1, 0.1, 0.3).into()),
                    ..Default::default()
                })
                .insert_bundle(RigidBodyBundle {
                    position: RigidBodyPosition {
                        position: ball_isometry,
                        ..Default::default()
                    },
                    ..Default::default()
                })
                .insert_bundle(ColliderBundle {
                    shape: ColliderShape::ball(rad),
                    ..Default::default()
                })
                .insert(Transform::default())
                .insert(ColliderPositionSync::Discrete)
                .insert(ColliderDebugRender::with_id(i))
                .id();

            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint = BallJoint::new(Point::origin(), point![0.0, 0.0, -shift * 2.0]);
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_handle,
                    ball_entity,
                ),));
            }
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = BallJoint::new(Point::origin(), point![-shift, 0.0, 0.0]);
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_handle,
                    ball_entity,
                ),));
            }
            body_handles.push(ball_entity);
        }
    }
}
