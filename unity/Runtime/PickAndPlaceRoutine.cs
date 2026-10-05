using System;
using System.IO;
using System.Linq;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Explicit, once-only replay of operator-taught joint commands; never auto-arms hardware.</summary>
    [DisallowMultipleComponent]
    public sealed class PickAndPlaceRoutine : MonoBehaviour
    {
        public RobotArm arm;
        public bool automaticUpdate=true;
        public string storagePath;
        public bool IsPlaying {get;private set;}
        public bool PlayingOnHardware {get;private set;}
        public int CurrentStepIndex {get;private set;}=-1;
        public string Status {get;private set;}="Move to a position, then add a recording.";
        public int Count=>program.waypoints.Count;
        public bool HasRecordings=>Count>0;
        public PickAndPlaceProgram ProgramSnapshot=>PickAndPlaceProgramStore.DeepClone(program);
        public string ProgramPath=>string.IsNullOrWhiteSpace(storagePath) ? Path.Combine(Application.persistentDataPath,"pick-and-place-program.json") : storagePath;
        private PickAndPlaceProgram program=new PickAndPlaceProgram(),playback;
        private RobotArm subscribedArm;
        private bool issuingOwnCommand,pausing;
        private float pauseRemaining;
        private static RobotArm routineStopArm;
        public static bool IsRoutineStopFor(RobotArm robot)=>robot!=null&&routineStopArm==robot;
        private void Start(){if(arm==null)arm=GetComponent<RobotArm>();Bind();}
        private void OnEnable(){Bind();}
        private void OnDisable(){if(IsPlaying)Stop();Unbind();}
        private void OnApplicationFocus(bool focused){if(!focused&&IsPlaying)Stop();}
        private void LateUpdate(){if(automaticUpdate)Tick(Time.deltaTime);}
        public void Bind()
        {
            if(subscribedArm==arm)return; Unbind();subscribedArm=arm;
            if(arm!=null){arm.SimulationStopped+=OnArmStopped;arm.MotionCommandIssued+=OnMotionCommand;}
        }
        private void Unbind()
        {if(subscribedArm!=null){subscribedArm.SimulationStopped-=OnArmStopped;subscribedArm.MotionCommandIssued-=OnMotionCommand;}subscribedArm=null;}
        public bool AddRecording(float dwellSeconds,out string reason)
        {
            int number=Count+1;
            while(program.waypoints.Any(s=>s.name=="Recording "+number))number++;
            return Capture("Recording "+number,dwellSeconds,out reason);
        }
        public bool RemoveRecording(int index,out string reason)
        {
            if(IsPlaying)return Fail("Stop playback before editing recordings.",out reason);
            if(index<0||index>=Count)return Fail("Select a recording first.",out reason);
            var candidate=PickAndPlaceProgramStore.DeepClone(program);
            string name=candidate.waypoints[index].name;candidate.waypoints.RemoveAt(index);
            if(!PickAndPlaceProgramStore.Save(ProgramPath,candidate,out reason)){Status=reason;return false;}
            program=candidate;Status="Removed and saved: "+name+".";return true;
        }
        public bool Capture(string name,float dwellSeconds,out string reason)
        {
            if(IsPlaying)return Fail("Stop playback before recording a waypoint.",out reason);
            if(arm==null)return Fail("Assign the arm first.",out reason);
            var candidate=PickAndPlaceProgramStore.DeepClone(program);
            var step=new PickAndPlaceWaypoint{name=name,angles=arm.CurrentAngles(),dwellSeconds=dwellSeconds};
            int index=candidate.waypoints.FindIndex(s=>s.name==name);
            if(index<0)candidate.waypoints.Add(step);else candidate.waypoints[index]=step;
            if(!PickAndPlaceProgramStore.Save(ProgramPath,candidate,out reason)){Status=reason;return false;}
            program=candidate;Status="Recorded and saved: "+name+" (current model commands).";return true;
        }
        public bool Save(string path,out string reason)
        {bool ok=PickAndPlaceProgramStore.Save(path,program,out reason);Status=ok?"Program saved.":reason;return ok;}
        public bool Load(string path,out string reason)
        {
            if(IsPlaying)return Fail("Stop playback before loading a program.",out reason);
            if(!PickAndPlaceProgramStore.Load(path,out var candidate,out reason)){Status=reason;return false;}
            program=candidate;Status="Program loaded. Nothing moves until Play once.";return true;
        }
        public bool PlayOnce(bool hardware,out string reason)
        {
            Bind();
            if(IsPlaying)return Fail("The program is already playing.",out reason);
            if(!PickAndPlaceProgramStore.Validate(program,out reason)){Status=reason;return false;}
            if(!HasRecordings)return Fail("Add at least one recording before playing.",out reason);
            var link=arm!=null?arm.GetComponent<RobotHardwareLink>():null;
            if(!hardware&&link!=null&&link.FollowActive)return Fail("Hold Robot link before simulation-only playback.",out reason);
            foreach(var step in program.waypoints)
                if(!ValidatePose(step.angles,hardware,out reason)){Status=step.name+": "+reason;reason=Status;return false;}
            RoutineStop(()=>
            {
                foreach(var ik in FindObjectsByType<InverseKinematics>(FindObjectsSortMode.None))if(ik.arm==arm)ik.Cancel();
                foreach(var plan in FindObjectsByType<MotionPlanner>(FindObjectsSortMode.None))if(plan.arm==arm&&plan.IsPlaying)plan.Stop();
                foreach(var sequence in FindObjectsByType<PoseSequence>(FindObjectsSortMode.None))if(sequence.arm==arm&&sequence.IsPlaying)sequence.Stop();
                arm.Stop();
            });
            arm.instantMode=false;foreach(var joint in arm.joints)joint.instantMode=false;
            playback=PickAndPlaceProgramStore.DeepClone(program);PlayingOnHardware=hardware;IsPlaying=true;CurrentStepIndex=0;
            BeginStep(); reason=null;return true;
        }
        private bool ValidatePose(float[] angles,bool hardware,out string reason)
        {
            if(arm==null||arm.joints==null||arm.joints.Length!=5||arm.joints.Any(j=>j==null)||angles==null||angles.Length!=5)
                return Fail("Five configured joints are required.",out reason);
            for(int i=0;i<5;i++)if(!JointMotion.IsFinite(angles[i])||angles[i]<arm.joints[i].minimumAngle||angles[i]>arm.joints[i].maximumAngle)
                return Fail("Waypoint exceeds current joint bounds: "+arm.joints[i].jointName+". Re-teach it; no clamping is applied.",out reason);
            if(hardware)
            {
                var link=arm.GetComponent<RobotHardwareLink>();
                if(link==null)return Fail("Connect and enable Robot link before real-arm playback.",out reason);
                return link.TryValidatePlaybackPose(angles,out reason);
            }
            reason=null;return true;
        }
        public void Tick(float deltaTime)
        {
            Bind();if(!IsPlaying||!JointMotion.IsFinite(deltaTime)||deltaTime<0)return;
            // Recheck the whole remaining program if execution bounds or link state change.
            for(int i=CurrentStepIndex;i<playback.waypoints.Count;i++)
                if(!ValidatePose(playback.waypoints[i].angles,PlayingOnHardware,out string reason))
                {Stop();Status="Playback stopped: "+reason;return;}
            if(!pausing)
            {
                if(!arm.IsAtTarget)return;
                // A model can arrive before the next USB pose update. Await fresh
                // reported commands before dwell/next step/final HOLD; this is not shaft feedback.
                if(PlayingOnHardware&&!arm.GetComponent<RobotHardwareLink>().IsPlaybackPoseReported(playback.waypoints[CurrentStepIndex].angles))return;
                pausing=true;pauseRemaining=playback.waypoints[CurrentStepIndex].dwellSeconds;
            }
            else pauseRemaining-=deltaTime;
            if(pauseRemaining>0)return;
            CurrentStepIndex++;
            if(CurrentStepIndex>=playback.waypoints.Count)
            {
                bool wasHardware=PlayingOnHardware;Cancel("Program complete; holding final position.");
                if(wasHardware)arm.GetComponent<RobotHardwareLink>().Hold("Pick-and-place program complete");
                return;
            }
            BeginStep();
        }
        private void BeginStep()
        {
            pausing=false;pauseRemaining=0;issuingOwnCommand=true;
            try{arm.MoveToPose(playback.waypoints[CurrentStepIndex].angles);}
            finally{issuingOwnCommand=false;}
            Status=(PlayingOnHardware?"REAL ARM":"SIMULATION ONLY")+" — "+(CurrentStepIndex+1)+"/"+playback.waypoints.Count+": "+playback.waypoints[CurrentStepIndex].name;
        }
        public void Stop(){Cancel("Playback stopped; press Play once to restart.");if(arm!=null)arm.Stop();}
        public void CancelForManualControl()
        {if(!IsPlaying)return;Cancel("Manual control cancelled playback.");RoutineStop(()=>arm.Stop());}
        private void OnMotionCommand(){if(!issuingOwnCommand&&IsPlaying)CancelForManualControl();}
        private void OnArmStopped(){if(IsPlaying&&!IsRoutineStopFor(arm))Cancel("Playback stopped; press Play once to restart.");}
        private void Cancel(string message){IsPlaying=false;PlayingOnHardware=false;CurrentStepIndex=-1;playback=null;pausing=false;pauseRemaining=0;Status=message;}
        private void RoutineStop(Action action)
        {var previous=routineStopArm;routineStopArm=arm;try{action();}finally{routineStopArm=previous;}}
        private bool Fail(string message,out string reason){Status=reason=message;return false;}
    }
}
