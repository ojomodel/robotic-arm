using System;
using UnityEngine;

namespace CareerFair.Robot
{
    [Serializable] public sealed class CameraLandmarks
    {
        public bool ready, hand;
        public int frame, version;
        public float age, hx, hy, palm, palmFacing;
        public string label, message, jpeg;
        public bool Usable => version==2 && ready && hand && age >= 0 && age < .4f && frame > 0 &&
            Valid(hx) && Valid(hy) && JointMotion.IsFinite(palm) && palm>.015f && palm<.8f &&
            JointMotion.IsFinite(palmFacing) && palmFacing>=.45f && palmFacing<=1.001f;
        private static bool Valid(float v) => JointMotion.IsFinite(v) && v >= 0 && v <= 1;
    }

    /// <summary>Approximate image-plane teleoperation, not metric camera calibration.</summary>
    public static class CameraTrackingMath
    {
        public static Vector3 HandTarget(CameraLandmarks sample, Vector2 center, Vector3 originMm, float spanMm,
            float referencePalm, float depthGain)
            => originMm + new Vector3(Mathf.Clamp((sample.hx-center.x)*spanMm,-140,140),
                Mathf.Clamp((center.y-sample.hy)*spanMm,-120,120),160+DepthOffset(sample.palm,referencePalm,depthGain));
        public static float DepthOffset(float palm,float referencePalm,float gain)
        {
            if(!JointMotion.IsFinite(palm)||palm<=0||!JointMotion.IsFinite(referencePalm)||referencePalm<=0||
                !JointMotion.IsFinite(gain)||gain<0) throw new ArgumentException("Invalid palm depth reference");
            return Mathf.Clamp((Mathf.Clamp(referencePalm/palm,.55f,1.8f)-1)*gain,-130,150);
        }
        // Keep a virtual stand-off from the tracked hand and point along its approach ray.
        public static Vector3 ToolTarget(Vector3 hand,Vector3 origin)
            => hand-(hand-(origin-Vector3.forward*200)).normalized*160;

        public static Vector3 Aim(ForwardKinematics.Snapshot snapshot, float[] q, Vector3 currentForward)
        {
            snapshot.EvaluateChain(q, out _, out var axes);
            for(int j=0;j<snapshot.controlledJointCount;j++)
                currentForward = Quaternion.AngleAxis((q[j]-snapshot.baseAngles[j])*snapshot.directionSigns[j],axes[j])*currentForward;
            return currentForward.normalized;
        }

        // Local damped least-squares with hand position weighted ahead of pointing.
        // Four motion joints cannot satisfy arbitrary position AND 3D orientation.
        public static float[] Step(ForwardKinematics.Snapshot snapshot, Vector3 forward, Vector3 handWorld,
            Vector3 headWorld, float maxStep, bool aimHead)
        {
            if(!ForwardKinematics.IsFinite(handWorld)||!ForwardKinematics.IsFinite(headWorld)||
                !ForwardKinematics.IsFinite(forward)||!JointMotion.IsFinite(maxStep)||maxStep<=0) throw new ArgumentException("Invalid camera target");
            var q=snapshot.Clamp(snapshot.baseAngles);
            const float epsilon=.05f;
            for(int iteration=0;iteration<12;iteration++)
            {
                var residual=Residual(snapshot,q,forward,handWorld,headWorld,aimHead);
                var jac=new double[6,4];
                for(int j=0;j<4;j++)
                {
                    var probe=(float[])q.Clone();
                    probe[j]=Mathf.Clamp(q[j]+epsilon,snapshot.minimumAngles[j],snapshot.maximumAngles[j]);
                    if(Mathf.Abs(probe[j]-q[j])<.001f) probe[j]=Mathf.Max(snapshot.minimumAngles[j],q[j]-epsilon);
                    float d=probe[j]-q[j]; if(Mathf.Abs(d)<.001f)continue;
                    var changed=Residual(snapshot,probe,forward,handWorld,headWorld,aimHead);
                    for(int k=0;k<6;k++) jac[k,j]=(changed[k]-residual[k])/d;
                }
                var normal=new double[4,5];
                for(int i=0;i<4;i++)
                {
                    for(int j=0;j<4;j++){for(int k=0;k<6;k++)normal[i,j]+=jac[k,i]*jac[k,j];if(i==j)normal[i,j]+=.025;}
                    for(int k=0;k<6;k++)normal[i,4]-=jac[k,i]*residual[k];
                }
                for(int i=0;i<4;i++)
                {
                    double divisor=normal[i,i];
                    for(int k=i;k<5;k++)normal[i,k]/=divisor;
                    for(int j=0;j<4;j++)if(i!=j){double factor=normal[j,i];for(int k=i;k<5;k++)normal[j,k]-=factor*normal[i,k];}
                }
                bool improved=false;
                for(float scale=1;scale>=.03125f;scale*=.5f)
                {
                    var next=(float[])q.Clone();
                    for(int j=0;j<4;j++)next[j]=Mathf.Clamp(q[j]+(float)normal[j,4]*scale,
                        Mathf.Max(snapshot.minimumAngles[j],snapshot.baseAngles[j]-maxStep),
                        Mathf.Min(snapshot.maximumAngles[j],snapshot.baseAngles[j]+maxStep));
                    if(Cost(Residual(snapshot,next,forward,handWorld,headWorld,aimHead))<Cost(residual)-.00001)
                    {q=next;improved=true;break;}
                }
                if(!improved)break;
            }
            return q;
        }
        private static double[] Residual(ForwardKinematics.Snapshot s,float[] q,Vector3 forward,Vector3 hand,Vector3 head,bool aim)
        {
            var tool=s.Evaluate(q); var position=(tool-hand)*s.millimetersPerWorldUnit;
            var pointing=aim ? (Aim(s,q,forward)-(head-tool).normalized)*25f : Vector3.zero;
            return new double[]{position.x,position.y,position.z,pointing.x,pointing.y,pointing.z};
        }
        private static double Cost(double[] v){double result=0;foreach(var n in v)result+=n*n;return result;}
    }
}
