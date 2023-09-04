// Tutorial: Rotation
// Destroy the asteroid. The target is in a random
// location given by the "target()" function.
//
// You can get the angle between a vector and the x-axis: vec2(a, b).angle()
// And compare an angle with your ship's heading: angle_diff(heading(), angle)
//
// If angle_diff returns a positive number you need to turn left, or if negative then right.
// The turn function takes a speed argument, where positive speeds result in turning left
// and negative speeds will turn right.
use oort_api::prelude::*;

pub struct Ship {}

impl Ship {
    pub fn new() -> Ship {
        Ship {}
    }

    pub fn tick(&mut self) {
        // Hint: "angle_diff(heading(), (target() - position()).angle())"
        // returns the direction your ship needs to turn to face the target.
        torque(10.0);

        fire(0);
    }
}

pub mod weapons {
    use super::targeting::TargetSystem;

    pub struct Weapons {
        targeting: TargetSystem,
    }

    impl Weapons {
        pub fn new(target_system: TargetSystem) -> Self {
            Weapons {
                targeting: target_system,
            }
        }
    }
}

pub mod targeting {
    use oort_api::{
        debug,
        prelude::{angle_diff, draw_diamond, heading, position, vec2, ScanResult, Vec2},
        Message,
    };

    use super::steering::HeadingControl;

    use super::{pidcontrol::PIDController, util};

    const BULLET_SPEED: f64 = 1000.0;

    pub struct Target {
        position: Vec2,
        velocity: Vec2,
    }

    impl Target {
        fn new(position: Vec2, velocity: Vec2) -> Target {
            Target { position, velocity }
        }
    }

    impl From<ScanResult> for Target {
        fn from(value: ScanResult) -> Self {
            Target::new(value.position, value.velocity)
        }
    }

    impl From<Message> for Target {
        fn from(value: Message) -> Self {
            let position = vec2(value[0], value[1]);
            let velocity = vec2(value[2], value[3]);

            Target::new(position, velocity)
        }
    }
    pub struct TargetSystem {
        angular_velocity_control: PIDController,
        heading_control: Box<dyn HeadingControl>,
        prev_target_velocity: Vec2,
    }

    impl TargetSystem {
        pub fn new<T: HeadingControl + 'static>(
            angular_velocity_control: PIDController,
            heading_control: T,
        ) -> Self {
            TargetSystem {
                angular_velocity_control,
                heading_control: Box::new(heading_control),
                prev_target_velocity: Vec2::default(),
            }
        }

        /// Calculate target lead and apply turn control
        pub fn target(&mut self, target: &Target) {
            let target_lead = self.lead_target(target);
            let error = self.angle_to_target(target_lead);
            let speed = self.angular_velocity_control.calculate_correction(error);
            self.heading_control.turn(speed);
        }

        fn angle_to_target(&self, target: Vec2) -> f64 {
            let displacement = target - position();
            let angle_error = angle_diff(heading(), displacement.angle());
            debug!("Angle error: {angle_error}");
            angle_error
        }

        /// Use the third kinematic equation to predict the position of a target given its velocity and acceleration
        pub fn lead_target(&mut self, target: &Target) -> Vec2 {
            // estimate time term of the third kinematic equation
            let t = self.time_to_target(&target);

            // get displacement of target over "t" given current velocity
            let displacement_v = target.velocity * t;

            // get target's acceleration
            let target_accel = target.velocity - self.prev_target_velocity;
            self.prev_target_velocity = target.velocity;

            // get displacement of target over "t" given current acceleration
            let displacement_a = target_accel * t.powi(2) / 2.0;

            // combine displacement terms with current target position to get predicted position
            let target_lead = displacement_v + displacement_a + target.position;

            draw_diamond(target_lead, 50.0, util::rgb(255, 0, 0));
            target_lead
        }

        /// Given the current distance to the target, return the amount of time it will take for
        /// a projectile to reach the target. This doesn't account for the target's velocity or acceleration
        /// because we can only solve for one variable of the kinematic equation at a time. Hence, this only serves as
        /// an approximation of the time of flight
        fn time_to_target(&self, target: &Target) -> f64 {
            let displacement = target.position - position();
            let t = displacement.length() / BULLET_SPEED;
            debug!("Target Distance: {displacement}");
            debug!("Time to target: {t}");
            t
        }
    }
}

pub mod radar {
    use oort_api::prelude::*;

    #[derive(Default)]
    pub struct Radar;

    impl Radar {
        pub fn scan() -> Option<ScanResult> {
            match scan() {
                Some(t) => Some(t),
                None => {
                    let heading = radar_heading() + radar_width();
                    set_radar_heading(heading);
                    None
                }
            }
        }
    }
}

pub mod pidcontrol {

    use oort_api::debug;

    /// A standard linear PID controller
    ///
    /// applies proportional, integral and derivative terms to calculate corrective action.
    pub struct PIDController {
        proportional_weight: f64,
        integral_weight: f64,
        integral_accumulator: f64,
        integral_accumulator_limit: f64,
        prev_error: f64,
        derivative_weight: f64,
    }

    impl PIDController {
        pub fn new(
            proportional_weight: f64,
            integral_weight: f64,
            integral_accumulator_limit: f64,
            derivative_weight: f64,
        ) -> PIDController {
            PIDController {
                proportional_weight,
                integral_weight,
                integral_accumulator: f64::default(),
                integral_accumulator_limit,
                prev_error: f64::default(),
                derivative_weight,
            }
        }

        /// Calculate the weighted proportional term
        fn term_p(&self, error: f64) -> f64 {
            let p = self.proportional_weight * error;
            debug!("P: {p}");
            p
        }

        /// Calculate the weighted integral term
        fn term_i(&mut self, error: f64) -> f64 {
            let mut i = self.integral_weight * error + self.integral_accumulator;

            // Apply anti wind-up logic
            if i.abs() >= self.integral_accumulator_limit {
                i = self.integral_accumulator_limit * i.signum();
            }
            debug!("I: {i}");
            self.integral_accumulator = i;
            i
        }

        /// Calculate the weighted derivative term
        fn term_d(&mut self, error: f64) -> f64 {
            let d = self.derivative_weight * (self.prev_error - error);
            debug!("D: {d}");
            self.prev_error = error;
            d
        }

        /// Calculate the necessary corrective action to apply to the control variable
        pub fn calculate_correction(&mut self, error: f64) -> f64 {
            let p = self.term_p(error);
            let i = self.term_i(error);
            let d = self.term_d(error);

            let pid = p + i + d;
            debug!("Correction: {pid}");
            pid
        }
    }
}

pub mod steering {

    use oort_api::{
        debug,
        prelude::{angular_velocity, torque, turn},
    };

    /// Use turn control by applying [torque()]
    pub struct TorqueControl {
        angular_velocity_limit: f64,
    }

    impl TorqueControl {
        pub fn new(angular_velocity_limit: f64) -> TorqueControl {
            TorqueControl {
                angular_velocity_limit,
            }
        }
    }

    impl HeadingControl for TorqueControl {
        fn turn(&self, speed: f64) {
            // enforce angular speed limit
            let mut speed = speed;
            if speed.abs() >= self.angular_velocity_limit {
                speed = self.angular_velocity_limit
            }

            // determine angular impulse required to achieve desired angular velocity
            let av_difference = speed - angular_velocity();
            debug!("Angular Velocity: {av_difference}");
            torque(av_difference);
        }
    }

    /// Use turn control by applying [turn()]
    #[derive(Default)]
    pub struct TurnControl;

    impl HeadingControl for TurnControl {
        fn turn(&self, speed: f64) {
            turn(speed)
        }
    }

    pub trait HeadingControl {
        fn turn(&self, speed: f64);
    }
}

pub mod util {
    pub fn rgb(r: u8, g: u8, b: u8) -> u32 {
        (r as u32) << 16 | (g as u32) << 8 | b as u32
    }
}
