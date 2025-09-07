using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Xml;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    public partial class Program : MyGridProgram
    {
        // This file contains your actual script.
        //
        // You can either keep all your code here, or you can create separate
        // code files to make your program easier to navigate while coding.
        //
        // Go to:
        // https://github.com/malware-dev/MDK-SE/wiki/Quick-Introduction-to-Space-Engineers-Ingame-Scripts
        //
        // to learn more about ingame scripts.

        string COCKPIT_NAME = "Cockpit";

        double dt;

        double Kv = 0.5;
        double Ka = 2;
        double Kj = 40;
        double Komega = 20;

        double MAX_SPEED = 100;
        double MAX_EFFECTIVE_THRUST = 0; // To be Set programatically
        double MAX_OMEGA = Math.PI;

        bool useDeltaInputs = true;
        double deltaXShipCoords = 0;
        double deltaYShipCoords = 0;
        double deltaZShipCoords = 10;

        IMyCockpit cockpit = null;
        List<IMyThrust> forwardThrusters = new List<IMyThrust>();
        List<IMyGyro> gyros = new List<IMyGyro>();

        Vector3D desiredPosition;
        Vector3D velMinus1 = new Vector3D(0, 0, 0);

        // Constructor
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1;

            // Get cockpit
            cockpit = (IMyCockpit)GridTerminalSystem.GetBlockWithName(COCKPIT_NAME);

            // Get forward thrusters
            GridTerminalSystem.GetBlocksOfType(forwardThrusters, (thruster) => thruster.Orientation == cockpit.Orientation);
            MAX_EFFECTIVE_THRUST = 0;
            foreach (IMyThrust thruster in forwardThrusters)
            {
                MAX_EFFECTIVE_THRUST += thruster.MaxEffectiveThrust;
            }

            // Get gyros
            GridTerminalSystem.GetBlocksOfType(gyros);
            foreach (IMyGyro gyro in gyros)
            {
                gyro.GyroOverride = true;
            }

            if (useDeltaInputs)
            {
                Vector3D position = cockpit.GetPosition();
                Vector3D deltaPositionShipCoords = new Vector3D(deltaXShipCoords, deltaYShipCoords, deltaZShipCoords);
                Vector3D deltaPosition = Vector3D.TransformNormal(deltaPositionShipCoords, cockpit.WorldMatrix);
                desiredPosition = position + deltaPosition;
            }
        }

        public void Save()
        {
        }

        // Main called every update period
        public void Main(string argument, UpdateType updateSource)
        {
            dt = (Runtime.LastRunTimeMs + Runtime.TimeSinceLastRun.Milliseconds) / 1000;

            // Get initial values
            Vector3D pos0 = cockpit.GetPosition();
            Vector3D vel0 = cockpit.GetShipVelocities().LinearVelocity;
            Vector3D acc0 = (vel0 - velMinus1) / dt;
            double mass = cockpit.CalculateShipMass().PhysicalMass;
            double thrustForceMag0 = 0;
            foreach (IMyThrust thruster in forwardThrusters)
            {
                thrustForceMag0 += thruster.CurrentThrust;
            }
            Vector3D thrustForce0 = thrustForceMag0 * cockpit.WorldMatrix.Forward;

            // Thrust Vector Controller
            Vector3D velocityRefUncapped = -Kv * (pos0 - desiredPosition);
            double velocityRefMag = Math.Min(velocityRefUncapped.Length(), MAX_SPEED);
            Vector3D velocityRef = velocityRefMag * velocityRefUncapped.Normalized();

            Vector3D accelRef = -Ka * (vel0 - velocityRef);
            Vector3D jerkRef = -Kj * (acc0 - accelRef);
            Vector3D thrustForceDotRef = mass * jerkRef;

            Vector3D thrustForceRef = thrustForce0 + thrustForceDotRef * dt;
            double thrustForceMagInUncapped = thrustForceRef.Dot(cockpit.WorldMatrix.Forward);
            double thrustForceMagIn = Math.Min(Math.Max(thrustForceMagInUncapped, 0), MAX_EFFECTIVE_THRUST);

            // Gyro Controller
            // Get thrust vector in ship coords
            Vector3D thrustForceRefShipCoords = Vector3D.TransformNormal(thrustForceRef, MatrixD.Transpose(cockpit.WorldMatrix));

            // Delta Theta in ship coords
            Vector3D forwardDirShipCoords = new Vector3D(1, 0, 0);
            Vector3D deltaThetaTempShipCoords = forwardDirShipCoords.Cross(thrustForceRefShipCoords.Normalized());
            double deltaThetaMag = Math.Asin(deltaThetaTempShipCoords.Length());
            Vector3D deltaThetaShipCoords = deltaThetaMag * deltaThetaTempShipCoords.Normalized();

            Vector3D omegaInShipCoordsUncapped = Komega * deltaThetaShipCoords;
            double omegaInMag = Math.Min(omegaInShipCoordsUncapped.Length(), MAX_OMEGA);
            Vector3D omegaInShipCoords = omegaInMag * omegaInShipCoordsUncapped.Normalized();

            // Set Inputs
            float thrustOverridePercentage = (float)(thrustForceMagIn / MAX_EFFECTIVE_THRUST);
            foreach (IMyThrust thruster in forwardThrusters) {
                thruster.ThrustOverridePercentage = thrustOverridePercentage;
            }

            foreach (IMyGyro gyro in gyros) {
                gyro.GyroOverride = true;
            }





            velMinus1 = vel0;
        }
    }

    // for n = steps
    //     p           = max([n - 1, 1]);
    //     x0          = x(:, p);
    //     v0          = v(:, p);
    //     a0          = a(:, p);
    //     F_thrust0   = F_thrust_mag(p);
    //     B0          = B(:, :, p);

    //     % Control
    //     v_ref_uncapped = -K_v * (x0 - x_ref);
    //     v_ref_mag = min(norm(v_ref_uncapped), max_speed);
    //     v_ref = v_ref_mag * v_ref_uncapped / norm(v_ref_uncapped);

    //     a_ref = -K_a * (v0 - v_ref);

    //     j_ref = -K_j * (a0 - a_ref);
    //     F_thrust_dot_ref = m * j_ref;

    //     F_thrust_ref = F_thrust0 * B0(:, 1) + F_thrust_dot_ref * dt;
    //     F_thrust_mag_in_uncapped = dot(F_thrust_ref, B0(:, 1));
    //     F_thrust_mag_in = min(max(F_thrust_mag_in_uncapped, 0), max_thrust);

    //     % Get thrust vector in ship coords
    //     F_thrust_ref_s = B0 \ F_thrust_ref;
    //     F_thrust_ref_dir_s = F_thrust_ref_s / norm(F_thrust_ref_s);
    //     b_1_s = [1; 0; 0]; % Forward direction in ship coords

    //     % Delta theta in ship coords
    //     delta_theta_temp_s = cross(b_1_s, F_thrust_ref_dir_s);
    //     delta_theta_mag = asin(norm(delta_theta_temp_s));
    //     delta_theta_s = delta_theta_mag * delta_theta_temp_s / norm(delta_theta_temp_s);

    //     omega_in_uncapped_s = K_omega * delta_theta_s;
    //     omega_in_mag = min(norm(omega_in_uncapped_s), max_omega);
    //     omega_in_s = omega_in_mag * omega_in_uncapped_s / norm(omega_in_uncapped_s);

    //     % Update state
    //     F_net   = F_thrust_mag_in * B0(:, 1) + F_g;
    //     a(:, n) = F_net / m;
    //     v(:, n) = v0 + a(:, n) * dt;
    //     x(:, n) = x0 + v0 * dt + 0.5 * a(:, n) * dt * dt;
    //     F_thrust_mag(n) = F_thrust_mag_in;

    //     dtheta_s = omega_in_s * dt;
    //     R_roll = [1 0 0; 0 cos(dtheta_s(1)) -sin(dtheta_s(1)); 0 sin(dtheta_s(1)) cos(dtheta_s(1))];
    //     R_pitch = [cos(dtheta_s(2)) 0 sin(dtheta_s(2)); 0 1 0; -sin(dtheta_s(2)) 0 cos(dtheta_s(2))];
    //     R_yaw = [cos(dtheta_s(3)) -sin(dtheta_s(3)) 0; sin(dtheta_s(3)) cos(dtheta_s(3)) 0; 0 0 1];
    //     B(:, :, n) = B0 * R_roll * R_pitch * R_yaw;
    //     b1(:, n) = B(:, 1, n);
    //     omega_s(:, n) = omega_in_s;
    // end
}
