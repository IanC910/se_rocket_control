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
using System.Runtime.InteropServices.WindowsRuntime;
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




        string COCKPIT_NAME = "Cockpit";

        double dt;

        double Kv = 0.3;
        double Ka = 2;
        double Kj = 10;
        double Komega = 6;

        double MAX_SPEED = 100;
        double MAX_EFFECTIVE_THRUST = 0; // To be Set programatically
        double MAX_OMEGA = Math.PI;

        bool useDeltaInputs = true;
        double deltaXShipCoords = 0;
        double deltaYShipCoords = 0;
        double deltaZShipCoords = 300;

        IMyCockpit cockpit = null;
        List<IMyThrust> forwardThrusters = new List<IMyThrust>();
        List<IMyGyro> gyros = new List<IMyGyro>();

        Vector3D desiredPosition;
        Vector3D velMinus1 = new Vector3D(0, 0, 0);
        Vector3D velMinus2 = new Vector3D(0, 0, 0);

        // Constructor
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1;

            // Get cockpit
            cockpit = (IMyCockpit)GridTerminalSystem.GetBlockWithName(COCKPIT_NAME);

            // Get forward thrusters
            Base6Directions.Direction thrusterDirection = Base6Directions.Direction.Forward;
            switch (cockpit.Orientation.Forward)
            {
                case Base6Directions.Direction.Forward: thrusterDirection = Base6Directions.Direction.Backward; break;
                case Base6Directions.Direction.Backward: thrusterDirection = Base6Directions.Direction.Forward; break;
                case Base6Directions.Direction.Left: thrusterDirection = Base6Directions.Direction.Right; break;
                case Base6Directions.Direction.Right: thrusterDirection = Base6Directions.Direction.Left; break;
                case Base6Directions.Direction.Up: thrusterDirection = Base6Directions.Direction.Down; break;
                case Base6Directions.Direction.Down: thrusterDirection = Base6Directions.Direction.Up; break;
                default: break;
            }
            GridTerminalSystem.GetBlocksOfType(forwardThrusters, (thruster) => thruster.Orientation.Forward == thrusterDirection);
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
            Vector3D acc0 = (3*vel0 - 4*velMinus1 + velMinus2) / (2*dt);
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
            // thrustForceRefShipCoords = cockPit.WorldMatrix.inverse * thrustForceRef
            Vector3D thrustForceRefShipCoords = Vector3D.TransformNormal(thrustForceRef, MatrixD.Transpose(cockpit.WorldMatrix));

            // Delta Theta in ship coords
            Vector3D forwardVecShipCoords = Vector3D.TransformNormal(cockpit.WorldMatrix.Forward, MatrixD.Transpose(cockpit.WorldMatrix));
            Vector3D deltaThetaTempShipCoords = forwardVecShipCoords.Cross(thrustForceRefShipCoords.Normalized());
            double deltaThetaMag = Math.Asin(deltaThetaTempShipCoords.Length());
            Vector3D deltaThetaShipCoords = deltaThetaMag * deltaThetaTempShipCoords.Normalized();

            Vector3D omegaInShipCoordsUncapped = Komega * deltaThetaShipCoords;
            double omegaInMag = Math.Min(omegaInShipCoordsUncapped.Length(), MAX_OMEGA);
            Vector3D omegaInShipCoords = omegaInMag * omegaInShipCoordsUncapped.Normalized();

            // Set Inputs
            cockpit.DampenersOverride = false;
            float thrustOverridePercentage = (float)(thrustForceMagIn / MAX_EFFECTIVE_THRUST);
            foreach (IMyThrust thruster in forwardThrusters) {
                thruster.ThrustOverridePercentage = thrustOverridePercentage;
            }

            foreach (IMyGyro gyro in gyros)
            {
                gyro.Roll = -(float)omegaInShipCoords.Z;
                gyro.Pitch = -(float)omegaInShipCoords.X;
                gyro.Yaw = -(float)omegaInShipCoords.Y;


                // switch (gyro.Orientation.Forward)
                // {
                //     case Base6Directions.Direction.Forward:
                //     case Base6Directions.Direction.Backward:
                //     case Base6Directions.Direction.Left:
                //     case Base6Directions.Direction.Right:
                //     case Base6Directions.Direction.Up:
                //     case Base6Directions.Direction.Down:
                //     default: break;
                // }
            }

            // Set vars for next loop
            velMinus2 = velMinus1;
            velMinus1 = vel0;
        }
    }
}
