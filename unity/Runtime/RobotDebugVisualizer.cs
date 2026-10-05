using UnityEngine;

namespace CareerFair.Robot
{
    public class RobotDebugVisualizer : MonoBehaviour
    {
        public RobotArm arm;
        public RobotCollisionMonitor collisionMonitor;
        public Material lineMaterial;
        public bool showAxes = true, showPivots = true, showOrigin = true;
        public bool showTarget = true, showEndEffector = true, showTrajectory = true, showColliderBounds;
        public bool showJointLimits;
        public float axisLength = .025f;
        [HideInInspector] public Vector3[] trajectory = new Vector3[0];
        [HideInInspector] public Color trajectoryColor = new Color(.3f,.85f,1f);
        public Transform[] inspectableGeometry;

        void OnRenderObject()
        {
            if (!arm || !lineMaterial) return;
            lineMaterial.SetPass(0);
            GL.PushMatrix(); GL.MultMatrix(Matrix4x4.identity); GL.Begin(GL.LINES);
            if (showOrigin && arm.robotOrigin) Axes(arm.robotOrigin.position, arm.robotOrigin.rotation, axisLength*1.5f);
            foreach (var j in arm.joints)
            {
                if (!j) continue;
                if (showAxes) Axes(j.transform.position, j.transform.rotation, axisLength);
                if (showPivots) Cross(j.transform.position,.003f, Color.white);
                if (showJointLimits)
                {
                    var basis=(j.transform.parent ? j.transform.parent.rotation : Quaternion.identity)*j.baselineLocalRotation;
                    var axis=basis*j.NormalizedAxis;
                    var u=Vector3.Cross(axis,basis*Vector3.up);
                    if(u.sqrMagnitude<.01f) u=Vector3.Cross(axis,basis*Vector3.right);
                    u=u.normalized*axisLength*1.1f;
                    float direction=j.reverseDirection?-1:1;
                    float start=(j.minimumAngle+j.angleOffset)*direction, end=(j.maximumAngle+j.angleOffset)*direction;
                    var prev=j.transform.position+Quaternion.AngleAxis(start,axis)*u;
                    for(int k=1;k<=36;k++)
                    {
                        var next=j.transform.position+Quaternion.AngleAxis(Mathf.Lerp(start,end,k/36f),axis)*u;
                        Line(prev,next,new Color(1,.7f,.15f)); prev=next;
                    }
                    var tangent=Vector3.Cross(axis,u).normalized*direction;
                    var q=Quaternion.AngleAxis(j.AppliedAngle+25*direction,axis);
                    var tip=j.transform.position+q*u;
                    Line(tip,tip-q*tangent*.006f,Color.yellow);
                }
            }
            if(showEndEffector && arm.endEffector) Cross(arm.endEffector.position,.006f,Color.cyan);
            if(showTarget && arm.target && arm.endEffector)
            {
                Line(arm.endEffector.position, arm.target.position, new Color(.8f,.8f,.8f));
                Cross(arm.target.position,.009f,new Color(1,.6f,.12f));
            }
            if(showTrajectory && trajectory!=null)
                for(int i=1;i<trajectory.Length;i++) Line(trajectory[i-1],trajectory[i],trajectoryColor);
            if(showColliderBounds && collisionMonitor && collisionMonitor.proxies!=null)
            {
                foreach(var proxy in collisionMonitor.proxies)
                {
                    if(proxy==null || !proxy.enabled || !proxy.frame)continue;
                    try {BoxLines(collisionMonitor.GetWorldCorners(proxy),new Color(1,.6f,.2f,.65f));}
                    catch(System.ArgumentException) { } // Invalid proxy settings are reported by the monitor panel.
                }
            }
            else if(showColliderBounds && inspectableGeometry!=null)
                foreach(var t in inspectableGeometry) if(t && t.TryGetComponent<Renderer>(out var r)) BoundsLines(r.bounds,new Color(1,.5f,.2f,.5f));
            GL.End(); GL.PopMatrix();
        }
        static void Line(Vector3 a,Vector3 b,Color c){GL.Color(c);GL.Vertex(a);GL.Vertex(b);}
        static void Cross(Vector3 p,float s,Color c){Line(p-Vector3.right*s,p+Vector3.right*s,c);Line(p-Vector3.up*s,p+Vector3.up*s,c);Line(p-Vector3.forward*s,p+Vector3.forward*s,c);}
        static void Axes(Vector3 p,Quaternion q,float s){Line(p,p+q*Vector3.right*s,Color.red);Line(p,p+q*Vector3.up*s,Color.green);Line(p,p+q*Vector3.forward*s,Color.blue);}
        static void BoxLines(Vector3[] p,Color c)
        {
            for(int i=0;i<4;i++){Line(p[i],p[(i+1)%4],c);Line(p[i+4],p[(i+1)%4+4],c);Line(p[i],p[i+4],c);}
        }
        static void BoundsLines(Bounds b,Color c)
        {
            var pts=new Vector3[8]; for(int i=0;i<8;i++)pts[i]=b.center+Vector3.Scale(b.extents,new Vector3((i&1)==0?-1:1,(i&2)==0?-1:1,(i&4)==0?-1:1));
            for(int i=0;i<8;i++)for(int d=0;d<3;d++){int n=i^(1<<d);if(n>i)Line(pts[i],pts[n],c);}
        }
    }
}
