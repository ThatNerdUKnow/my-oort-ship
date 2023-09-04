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
use pidcontrol::{DerivativeParams, IntegralParams, PIDController, ProportionalParams};
use steering::{TorqueControl, TurnControl};
use targeting::TargetSystem;
use weapons::Weapons;

pub struct Ship {
    weapons: Weapons,
}

/// This implementation only serves to get clippy to shut up
impl Default for Ship {
    fn default() -> Self {
        Self::new()
    }
}

impl Ship {
    pub fn new() -> Ship {
        //let angular_velocity_control = PIDController::new(3.0, 1.0, 1.0, 2.0);

        let angular_velocity_control = PIDController::new(
            ProportionalParams {
                weight: 3.0,
                pow: 1,
            },
            IntegralParams {
                weight: 1.0,
                pow: 1,
                limit: 0.5,
            },
            DerivativeParams {
                weight: 2.0,
                pow: 1,
            },
        );
        let turn_control = TorqueControl::new(3.0);
        //let turn_control = TurnControl::default();
        let targeting = TargetSystem::new(angular_velocity_control, turn_control);
        let weapons = Weapons::new(targeting, 0.1);
        Ship { weapons }
    }

    pub fn tick(&mut self) {
        // Hint: "angle_diff(heading(), (target() - position()).angle())"
        // returns the direction your ship needs to turn to face the target.
        self.weapons.search_and_destroy();
    }
}

pub mod weapons {

    use oort_api::prelude::fire;

    use super::radar::Radar;

    use super::targeting::TargetSystem;

    pub struct Weapons {
        targeting: TargetSystem,
        radar: Radar,
        fire_angle_threshold: f64,
    }

    impl Weapons {
        pub fn new(target_system: TargetSystem, fire_angle_threshold: f64) -> Self {
            Weapons {
                targeting: target_system,
                radar: Radar::default(),
                fire_angle_threshold,
            }
        }

        pub fn search_and_destroy(&mut self) {
            if let Some(target) = self.radar.scan() {
                let target = self.targeting.target(&target.into());
                let angle = self.targeting.angle_to_target(target);

                if angle.abs() <= self.fire_angle_threshold {
                    fire(0);
                }
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
        pub fn target(&mut self, target: &Target) -> Vec2 {
            let target_lead = self.lead_target(target);
            let error = self.angle_to_target(target_lead);
            let speed = self.angular_velocity_control.calculate_correction(error);
            self.heading_control.turn(speed);
            target_lead
        }

        pub fn angle_to_target(&self, target: Vec2) -> f64 {
            let displacement = target - position();
            let angle_error = angle_diff(heading(), displacement.angle());
            debug!("Angle error: {angle_error}");
            angle_error
        }

        /// Use the third kinematic equation to predict the position of a target given its velocity and acceleration
        fn lead_target(&mut self, target: &Target) -> Vec2 {
            // estimate time term of the third kinematic equation
            let mut t = self.time_to_target(target.position);

            let mut target_lead = self.lead_target_converge(target, t);

            let mut delta_t = 1.0;

            while delta_t > 0.0 {
                target_lead = self.lead_target_converge(target, t);
                let t_prime = self.time_to_target(target_lead);
                delta_t = t - t_prime;
                t = t_prime
            }

            //debug!("Target Distance: {displacement}");
            debug!("Time to target: {t}");

            self.prev_target_velocity = target.velocity;
            target_lead
        }

        fn lead_target_converge(&self, target: &Target, time: f64) -> Vec2 {
            // get displacement of target over time given current velocity
            let displacement_v = target.velocity * time;

            // get target's acceleration
            let target_accel = target.velocity - self.prev_target_velocity;

            // get displacement of target over time given current acceleration
            let displacement_a = target_accel * time.powi(2) / 2.0;

            // combine displacement terms with current target position to get predicted position
            let target_lead = displacement_v + displacement_a + target.position;
            draw_diamond(target_lead, 20.0, util::rgb(255, 0, 0));
            target_lead
        }

        /// Given the current distance to the target, return the amount of time it will take for
        /// a projectile to reach the target. This doesn't account for the target's velocity or acceleration
        /// because we can only solve for one variable of the kinematic equation at a time. Hence, this only serves as
        /// an approximation of the time of flight
        fn time_to_target(&self, target: Vec2) -> f64 {
            let displacement = target - position();

            displacement.length() / BULLET_SPEED
        }
    }
}

pub mod radar {
    use oort_api::prelude::*;

    use super::util;

    #[derive(Default)]
    pub struct Radar;

    impl Radar {
        pub fn scan(&self) -> Option<ScanResult> {
            match scan() {
                Some(t) => {
                    let angle = util::angle_to_target_global(t.position);
                    set_radar_heading(angle);
                    Some(t)
                }
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

    pub struct ProportionalParams {
        pub weight: f64,
        pub pow: i32,
    }

    pub struct IntegralParams {
        pub weight: f64,
        pub pow: i32,
        pub limit: f64,
    }

    pub struct DerivativeParams {
        pub weight: f64,
        pub pow: i32,
    }

    /// A standard PID controller
    ///
    /// applies proportional, integral and derivative terms to calculate corrective action.
    pub struct PIDController {
        proportional: ProportionalParams,
        integral: IntegralParams,
        derivative: DerivativeParams,
        integral_accumulator: f64,
        prev_error: f64,
    }

    impl PIDController {
        pub fn new(
            proportional: ProportionalParams,
            integral: IntegralParams,
            derivative: DerivativeParams,
        ) -> PIDController {
            PIDController {
                proportional,
                integral,
                derivative,
                integral_accumulator: f64::default(),
                prev_error: f64::default(),
            }
        }

        /// Calculate the weighted proportional term
        fn term_p(&self, error: f64) -> f64 {
            let error = self.error_pow(error, self.proportional.pow);
            let p = self.proportional.weight * error;
            debug!("P: {p}");
            p
        }

        /// Calculate the weighted integral term
        fn term_i(&mut self, error: f64) -> f64 {
            let error = self.error_pow(error, self.integral.pow);
            let mut i = self.integral.weight * error + self.integral_accumulator;

            // Apply anti wind-up logic
            i = i.clamp(-self.integral.limit, self.integral.limit);

            debug!("I: {i}");
            self.integral_accumulator = i;
            i
        }

        /// Calculate the weighted derivative term
        fn term_d(&mut self, error: f64) -> f64 {
            let error = self.error_pow(error, self.derivative.pow);
            let d = self.derivative.weight * (self.prev_error - error);
            debug!("D: {d}");
            self.prev_error = error;
            d
        }

        /// Apply power to error. accounts for sign change for powers of... powers of two
        fn error_pow(&self, error: f64, pow: i32) -> f64 {
            let sign = error.signum();

            let mut error = error.powi(pow);

            if pow % 2 == 0{
                error *= sign;
            }

            error
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
        prelude::{angular_velocity, max_angular_acceleration, torque, turn},
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
            let max = max_angular_acceleration() * self.angular_velocity_limit;
            torque(
                (speed.clamp(-max, max) - angular_velocity()).signum() * max_angular_acceleration(),
            );
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
    use oort_api::prelude::*;

    pub fn rgb(r: u8, g: u8, b: u8) -> u32 {
        (r as u32) << 16 | (g as u32) << 8 | b as u32
    }

    pub fn angle_to_target_global(target: Vec2) -> f64 {
        let displacement = target - position();
        displacement.angle()
    }
}
