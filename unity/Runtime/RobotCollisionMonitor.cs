using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Simulation advisory only: oriented mesh-bound proxies, no dynamics.</summary>
    [DefaultExecutionOrder(200)]
    [DisallowMultipleComponent]
    public sealed class RobotCollisionMonitor : MonoBehaviour
    {
        [Serializable] public class Proxy
        {
            public string id,group;
            public int stage;
            public Transform frame;
            public Vector3 localCenter,localSize;
            public Quaternion localRotation=Quaternion.identity;
            public bool enabled=true;
        }
        [Serializable] public class IgnoredPair { public string first,second; }
        public struct OrientedBox
        {
            public Vector3 center,halfExtents;
            public Quaternion rotation;
            public OrientedBox(Vector3 c,Quaternion q,Vector3 half){center=c;rotation=q;halfExtents=half;}
            public Vector3 Axis(int i)=>rotation*(i==0?Vector3.right:i==1?Vector3.up:Vector3.forward);
        }
        public RobotArm arm;
        public bool monitoringEnabled=true;
        [Tooltip("Optional per-face inset. Zero encloses the configured mesh bounds. A positive inset can hide contacts, so it is only an advisory tuning control.")]
        [Min(0f)] public float insetMillimeters=0f;
        [Tooltip("Ignore exact touch and overlaps shallower than this value; never a physical clearance certificate.")]
        [Min(0f)] public float contactToleranceMillimeters=.05f;
        [Min(.02f)] public float pollIntervalSeconds=.1f;
        public bool drawProxyBounds;
        public Proxy[] proxies=Array.Empty<Proxy>();
        public IgnoredPair[] ignoredPairs=Array.Empty<IgnoredPair>();
        public bool HasPotentialCollision { get; private set; }
        public string Status { get; private set; }="Collision advisory not configured";
        public string[] CurrentWarnings { get; private set; }=Array.Empty<string>();
        public int TestedPairs { get; private set; }
        public int PotentialPairCount { get; private set; }
        public bool HasInvalidProxies { get; private set; }
        RobotArm boundArm;
        float nextPoll;
        bool dirty=true;
        readonly HashSet<string> currentPairIds=new HashSet<string>();

        static readonly HashSet<string> MajorCodes=new HashSet<string>{
            "00","01","02","03","04","05","06","07","07B","08","09",
            "20","21","23","26","27","33","34","39","40","41","42","43","44","H1","H2","H3"};

        void OnEnable(){Bind();dirty=true;}
        void OnDisable(){if(boundArm)boundArm.PoseUpdated-=PoseChanged;boundArm=null;}
        void Start(){if(arm&&(proxies==null||proxies.Length==0))ConfigureFromScene(arm);}
        void Bind()
        {
            if(boundArm==arm)return;
            if(boundArm)boundArm.PoseUpdated-=PoseChanged;
            boundArm=arm;if(boundArm)boundArm.PoseUpdated+=PoseChanged;
        }
        void PoseChanged(){dirty=true;}
        void LateUpdate()
        {
            Bind();
            // After the linkage's LateUpdate. Periodic fallback also handles
            // direct Inspector/Transform changes without an arm pose event.
            if(Time.unscaledTime>=nextPoll&&(dirty||Time.unscaledTime>=nextPoll+pollIntervalSeconds))EvaluateNow();
        }
        static string Code(string name){int at=name.IndexOf(" - ",StringComparison.Ordinal);return at<0?name:name.Substring(0,at);}
        static int Stage(Transform t)
        {
            for(var p=t;p!=null;p=p.parent)
            {
                var j=p.GetComponent<RobotJoint>();if(!j)continue;
                if(j.jointName=="Base")return 1;
                if(j.jointName=="Shoulder")return 2;
                if(j.jointName=="Elbow")return 3;
                if(j.jointName=="Wrist"||j.jointName=="Gripper")return 4;
            }
            return 0;
        }
        static Quaternion LinkFrame(Vector3 along,Vector3 right,Quaternion fallback)
        {
            if(along.sqrMagnitude<1e-10f)return fallback;
            var forward=along.normalized;
            var up=Vector3.Cross(forward,right.normalized);
            return up.sqrMagnitude<1e-8f?fallback:Quaternion.LookRotation(forward,up.normalized);
        }
        /// <summary>Call at the imported home pose. Frames are derived from actual
        /// joint centers and mesh vertices; subsequent motion carries each proxy.</summary>
        public void ConfigureFromScene(RobotArm source)
        {
            if(!source)throw new ArgumentNullException(nameof(source));
            if(source.joints.Any(j=>Mathf.Abs(j.currentAngle-j.homeAngle)>.001f))
                throw new InvalidOperationException("Configure collision proxies at the CAD home pose.");
            arm=source;Bind();
            var shoulder=arm.FindJoint("Shoulder");var elbow=arm.FindJoint("Elbow");var wrist=arm.FindJoint("Wrist");
            if(!shoulder||!elbow||!wrist||!arm.robotOrigin)throw new InvalidOperationException("The CAD pivot hierarchy is incomplete.");
            Quaternion ground=arm.robotOrigin.rotation;
            var right=shoulder.transform.TransformDirection(shoulder.NormalizedAxis);
            var upperFrame=LinkFrame(elbow.transform.position-shoulder.transform.position,right,ground);
            var foreFrame=LinkFrame(wrist.transform.position-elbow.transform.position,right,ground);
            var result=new List<Proxy>();
            string[] groups={"Fixed","Base","Shoulder","Elbow","Wrist"};
            foreach(var filter in arm.GetComponentsInChildren<MeshFilter>(true))
            {
                string code=Code(filter.name);if(!MajorCodes.Contains(code)||!filter.sharedMesh)continue;
                int stage=Stage(filter.transform);Quaternion orientation=stage==2?upperFrame:stage>=3?foreFrame:ground;
                if(code.Length==2&&code[0]=='0'&&code[1]>='1'&&code[1]<='5')
                {
                    var radial=filter.transform.TransformPoint(filter.sharedMesh.bounds.center)-arm.robotOrigin.position;
                    radial=Vector3.ProjectOnPlane(radial,arm.robotOrigin.up);
                    if(radial.sqrMagnitude>1e-10f)orientation=Quaternion.LookRotation(radial,arm.robotOrigin.up);
                }
                var localRotation=Quaternion.Inverse(filter.transform.rotation)*orientation;
                var inverse=Quaternion.Inverse(localRotation);
                var vertices=filter.sharedMesh.vertices;if(vertices.Length==0)continue;
                var bounds=new Bounds(inverse*vertices[0],Vector3.zero);
                for(int i=1;i<vertices.Length;i++)bounds.Encapsulate(inverse*vertices[i]);
                result.Add(new Proxy{id=code,group=groups[stage],stage=stage,frame=filter.transform,
                    localCenter=localRotation*bounds.center,localSize=bounds.size,localRotation=localRotation});
            }
            if(result.Select(p=>p.id).Distinct().Count()!=result.Count)throw new InvalidOperationException("Duplicate collision proxy component code.");
            proxies=result.ToArray();dirty=true;EvaluateNow();
        }
        static bool Finite(Vector3 v)=>JointMotion.IsFinite(v.x)&&JointMotion.IsFinite(v.y)&&JointMotion.IsFinite(v.z);
        static bool Finite(Quaternion q)=>JointMotion.IsFinite(q.x)&&JointMotion.IsFinite(q.y)&&JointMotion.IsFinite(q.z)&&JointMotion.IsFinite(q.w);

        public static OrientedBox BoxForProxy(Proxy proxy,float insetMeters=0f)
        {
            if(proxy==null||!proxy.frame||!Finite(proxy.localCenter)||!Finite(proxy.localSize)||!Finite(proxy.localRotation))
                throw new ArgumentException("Missing or nonfinite proxy frame/bounds.");
            if(proxy.localSize.x<=0||proxy.localSize.y<=0||proxy.localSize.z<=0||!JointMotion.IsFinite(insetMeters)||insetMeters<0)
                throw new ArgumentException("Proxy size/inset must be positive and finite.");
            var center=proxy.frame.TransformPoint(proxy.localCenter);
            var x=proxy.frame.TransformVector(proxy.localRotation*Vector3.right*(proxy.localSize.x*.5f));
            var y=proxy.frame.TransformVector(proxy.localRotation*Vector3.up*(proxy.localSize.y*.5f));
            var z=proxy.frame.TransformVector(proxy.localRotation*Vector3.forward*(proxy.localSize.z*.5f));
            if(!Finite(center)||!Finite(x)||!Finite(y)||!Finite(z)||Mathf.Min(x.magnitude,y.magnitude,z.magnitude)<1e-9f)
                throw new ArgumentException("Degenerate/nonfinite proxy transform.");
            Vector3 half;Quaternion rotation;
            if(Mathf.Max(Mathf.Abs(Vector3.Dot(x.normalized,y.normalized)),Mathf.Abs(Vector3.Dot(x.normalized,z.normalized)),Mathf.Abs(Vector3.Dot(y.normalized,z.normalized)))>1e-4f)
            {
                // Nonuniformly scaled rotated hierarchy can shear a box. Its
                // world AABB is a conservative fallback, not an invalid OBB.
                half=new Vector3(Mathf.Abs(x.x)+Mathf.Abs(y.x)+Mathf.Abs(z.x),Mathf.Abs(x.y)+Mathf.Abs(y.y)+Mathf.Abs(z.y),Mathf.Abs(x.z)+Mathf.Abs(y.z)+Mathf.Abs(z.z));
                rotation=Quaternion.identity;
            }
            else {half=new Vector3(x.magnitude,y.magnitude,z.magnitude);rotation=Quaternion.LookRotation(z.normalized,y.normalized);}
            half-=Vector3.one*insetMeters;
            if(Mathf.Min(half.x,half.y,half.z)<=0)throw new ArgumentException("Inset erases a collision proxy; reduce its value.");
            return new OrientedBox(center,rotation,half);
        }
        /// <summary>World corners for runtime line rendering. Edges use
        /// 0-1-2-3-0,4-5-6-7-4,0-4,1-5,2-6,3-7.</summary>
        public Vector3[] GetWorldCorners(Proxy proxy)
        {
            var b=BoxForProxy(proxy,insetMillimeters*.001f);
            var signs=new[]{new Vector3(-1,-1,-1),new Vector3(1,-1,-1),new Vector3(1,1,-1),new Vector3(-1,1,-1),
                new Vector3(-1,-1,1),new Vector3(1,-1,1),new Vector3(1,1,1),new Vector3(-1,1,1)};
            return signs.Select(v=>b.center+b.rotation*Vector3.Scale(v,b.halfExtents)).ToArray();
        }
        /// <summary>15-axis oriented-box SAT. Exact touch is not penetration.</summary>
        public static bool Overlap(OrientedBox a,OrientedBox b,float contactToleranceMeters,out float minimumOverlapMeters)
        {
            if(!Finite(a.center)||!Finite(b.center)||!Finite(a.rotation)||!Finite(b.rotation)||!Finite(a.halfExtents)||!Finite(b.halfExtents)||
                Mathf.Min(a.halfExtents.x,a.halfExtents.y,a.halfExtents.z,b.halfExtents.x,b.halfExtents.y,b.halfExtents.z)<=0||
                !JointMotion.IsFinite(contactToleranceMeters)||contactToleranceMeters<0)throw new ArgumentException("Invalid SAT box or tolerance.");
            var aa=new[]{a.Axis(0),a.Axis(1),a.Axis(2)};var bb=new[]{b.Axis(0),b.Axis(1),b.Axis(2)};
            var axes=new List<Vector3>(15);axes.AddRange(aa);axes.AddRange(bb);
            for(int i=0;i<3;i++)for(int j=0;j<3;j++){var cross=Vector3.Cross(aa[i],bb[j]);if(cross.sqrMagnitude>1e-12f)axes.Add(cross.normalized);}
            minimumOverlapMeters=float.PositiveInfinity;
            foreach(var direction in axes)
            {
                var n=direction.normalized;
                float reach=0;
                for(int i=0;i<3;i++)reach+=a.halfExtents[i]*Mathf.Abs(Vector3.Dot(aa[i],n))+b.halfExtents[i]*Mathf.Abs(Vector3.Dot(bb[i],n));
                float penetration=reach-Mathf.Abs(Vector3.Dot(b.center-a.center,n));
                minimumOverlapMeters=Mathf.Min(minimumOverlapMeters,penetration);
                if(penetration<=contactToleranceMeters)return false;
            }
            return true;
        }
        bool Ignore(Proxy a,Proxy b)
        {
            if(a.stage==b.stage||Math.Abs(a.stage-b.stage)==1)return true;
            return ignoredPairs!=null&&ignoredPairs.Any(p=>p!=null&&((p.first==a.id&&p.second==b.id)||(p.first==b.id&&p.second==a.id)));
        }
        public void EvaluateNow()
        {
            dirty=false;nextPoll=Time.unscaledTime+Mathf.Max(.02f,pollIntervalSeconds);currentPairIds.Clear();
            HasPotentialCollision=false;HasInvalidProxies=false;TestedPairs=0;PotentialPairCount=0;
            if(!monitoringEnabled){CurrentWarnings=Array.Empty<string>();Status="Collision advisory disabled";return;}
            if(proxies==null||proxies.Length==0){CurrentWarnings=Array.Empty<string>();Status="Collision advisory not configured";return;}
            var warnings=new List<string>();var boxes=new Dictionary<int,OrientedBox>();
            for(int i=0;i<proxies.Length;i++)
            {
                var p=proxies[i];if(p==null||!p.enabled)continue;
                try {boxes[i]=BoxForProxy(p,insetMillimeters*.001f);}
                catch(Exception e){HasInvalidProxies=true;warnings.Add("Cannot assess "+p.id+": "+e.Message);}
            }
            var indices=boxes.Keys.ToArray();
            for(int i=0;i<indices.Length;i++)for(int j=i+1;j<indices.Length;j++)
            {
                var a=proxies[indices[i]];var b=proxies[indices[j]];if(Ignore(a,b))continue;
                TestedPairs++;
                if(Overlap(boxes[indices[i]],boxes[indices[j]],Mathf.Max(0,contactToleranceMillimeters)*.001f,out float depth))
                {
                    PotentialPairCount++;currentPairIds.Add(a.id);currentPairIds.Add(b.id);
                    warnings.Add(a.id+" / "+b.id+": oriented bounds overlap ("+(depth*1000).ToString("F2")+" mm proxy depth)");
                }
            }
            HasPotentialCollision=PotentialPairCount>0;CurrentWarnings=warnings.ToArray();
            Status=HasInvalidProxies?"Collision advisory incomplete — invalid proxy settings":HasPotentialCollision?
                PotentialPairCount+" potential nonadjacent contact(s) — bounds advisory":"No nonadjacent proxy overlap detected — advisory only";
        }
        void OnDrawGizmos()
        {
            if(!drawProxyBounds||proxies==null)return;
            var old=Gizmos.matrix;var color=Gizmos.color;
            foreach(var p in proxies)
            {
                if(p==null||!p.enabled||!p.frame)continue;
                try
                {
                    var b=BoxForProxy(p,insetMillimeters*.001f);Gizmos.color=currentPairIds.Contains(p.id)?new Color(1,.2f,.1f,.8f):new Color(.15f,.8f,1,.45f);
                    Gizmos.matrix=Matrix4x4.TRS(b.center,b.rotation,b.halfExtents*2);Gizmos.DrawWireCube(Vector3.zero,Vector3.one);
                }
                catch(ArgumentException){ }
            }
            Gizmos.matrix=old;Gizmos.color=color;
        }
    }
}
