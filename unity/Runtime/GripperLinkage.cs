using System;
using UnityEngine;

namespace CareerFair.Robot
{
    [Serializable] public class GripperBody
    {
        public Transform body;
        public Vector3 homePosition;
        public Quaternion homeRotation;
        public void Place(Vector3 pivot, Quaternion delta, Vector3 newPivot)
        {
            body.localPosition=newPivot+delta*(homePosition-pivot);
            body.localRotation=delta*homeRotation;
        }
    }
    [Serializable] public class GripperSide
    {
        public string name;
        public Vector3 a,b,c,d,jawTip;
        public float direction=1;
        public GripperBody[] gears,jaws,links;
        [HideInInspector] public Vector3 currentB,currentC,currentTip;
    }
    public class GripperLinkage : MonoBehaviour
    {
        public RobotArm arm;
        public RobotJoint driver;
        public Transform wristFrame;
        public Transform endEffector;
        public Vector3 axis;
        public GripperSide[] sides;
        public bool valid=true;
        public string status="Not evaluated";
        public float jawOpeningMillimeters;
        public float maximumClosureErrorMillimeters;
        RobotArm subscribedArm;
        public void Bind(){if(subscribedArm)subscribedArm.PoseUpdated-=Refresh;subscribedArm=arm;if(subscribedArm)subscribedArm.PoseUpdated+=Refresh;}
        void OnEnable(){Bind();}
        void OnDisable(){if(subscribedArm)subscribedArm.PoseUpdated-=Refresh;subscribedArm=null;}
        void LateUpdate(){Refresh();}
        public void Refresh(){if(driver)Apply(driver.AppliedAngle);}
        public bool Apply(float angle)
        {
            if(sides==null || sides.Length!=2 || !wristFrame || !endEffector || !JointMotion.IsFinite(angle))return false;
            valid=true; maximumClosureErrorMillimeters=0;
            var nextB=new Vector3[2];var nextC=new Vector3[2];var gearQ=new Quaternion[2];
            for(int i=0;i<2;i++)
            {
                var s=sides[i]; gearQ[i]=Quaternion.AngleAxis(angle*s.direction,axis);
                nextB[i]=s.a+gearQ[i]*(s.b-s.a);
                var bd=s.d-nextB[i];float distance=bd.magnitude;
                float jaw=(s.c-s.b).magnitude,link=(s.c-s.d).magnitude;
                if(distance<1e-8f || distance>jaw+link+1e-6f || distance<Mathf.Abs(jaw-link)-1e-6f){valid=false;break;}
                float x=(jaw*jaw-link*link+distance*distance)/(2*distance);
                float h2=jaw*jaw-x*x;
                if(h2< -1e-8f){valid=false;break;}
                var unit=bd/distance;var side=Vector3.Cross(axis.normalized,unit).normalized;
                var homeSide=Vector3.Cross(axis.normalized,(s.d-s.b).normalized).normalized;
                float branch=Vector3.Dot(s.c-s.b,homeSide)>=0?1:-1;
                nextC[i]=nextB[i]+unit*x+side*(Mathf.Sqrt(Mathf.Max(0,h2))*branch);
            }
            if(!valid){status="Gripper linkage cannot close at this angle; previous valid geometry retained";return false;}
            for(int i=0;i<2;i++)
            {
                var s=sides[i];s.currentB=nextB[i];s.currentC=nextC[i];
                var jawQ=Quaternion.AngleAxis(Vector3.SignedAngle(s.c-s.b,nextC[i]-nextB[i],axis),axis);
                var linkQ=Quaternion.AngleAxis(Vector3.SignedAngle(s.c-s.d,nextC[i]-s.d,axis),axis);
                foreach(var body in s.gears)body.Place(s.a,gearQ[i],s.a);
                foreach(var body in s.jaws)body.Place(s.b,jawQ,nextB[i]);
                foreach(var body in s.links)body.Place(s.d,linkQ,s.d);
                s.currentTip=nextB[i]+jawQ*(s.jawTip-s.b);
                maximumClosureErrorMillimeters=Mathf.Max(maximumClosureErrorMillimeters,
                    Mathf.Abs(Vector3.Distance(nextB[i],nextC[i])-Vector3.Distance(s.b,s.c))*1000,
                    Mathf.Abs(Vector3.Distance(s.d,nextC[i])-Vector3.Distance(s.d,s.c))*1000);
            }
            endEffector.position=wristFrame.TransformPoint((sides[0].currentTip+sides[1].currentTip)*.5f);
            jawOpeningMillimeters=Vector3.Distance(sides[0].currentTip,sides[1].currentTip)*1000;
            status="Two four-bar loops closed";return true;
        }
    }
}
