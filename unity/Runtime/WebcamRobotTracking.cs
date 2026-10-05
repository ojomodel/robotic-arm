using System;
using System.Collections;
using System.IO;
using System.Net;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.Networking;

namespace CareerFair.Robot
{
    [DefaultExecutionOrder(100)]
    public sealed class WebcamRobotTracking : MonoBehaviour
    {
        public RobotArm arm;
        public bool panelOpen;
        public bool Following { get; private set; }
        public string Status { get; private set; } = "Start the camera, then show one open palm.";
        public bool aimHand=true;
        public float spanMillimeters=400;
        public float depthGain=180;
        private float referencePalm;
        private CameraLandmarks sample;
        private Texture2D preview;
        private System.Diagnostics.Process helper;
        private Coroutine polling;
        private UnityWebRequest request;
        private string token, url, handLabel;
        private int cameraIndex, lastFrame;
        private float lastFresh, nextMove;
        private Vector2 center;
        private Vector3 origin, smoothHand;
        private bool ownCommand;
        private string pauseReason;
        private Rect panel=new Rect(20,115,360,575);
        private GUIStyle wrap;
        private Vector2 panelScroll;
        private Transform handMarker, toolMarker;
        private LineRenderer aimLine;
        private RobotHardwareLink Link => arm ? arm.GetComponent<RobotHardwareLink>() : null;
        private RobotControlPanel controlPanel;
        private Func<Vector2,bool> priorHitTest, hitTest;
        public bool HardwareBlocked => Link && (Link.Connected || Link.FollowActive);
        public bool HasFreshTracking => sample!=null && sample.Usable && Time.realtimeSinceStartup-lastFresh<.4f;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            var robot=FindFirstObjectByType<RobotArm>();
            if(robot && !robot.GetComponent<WebcamRobotTracking>()) robot.gameObject.AddComponent<WebcamRobotTracking>().arm=robot;
        }
        private void Start()
        {
            Debug.Log("WEBCAM_HAND_DEPTH_VERSION_2: hand-only data, estimated depth, simulation-only");
            if(!arm)arm=GetComponent<RobotArm>();
            arm.MotionCommandIssued+=ExternalMotion;
            arm.SimulationStopped+=ExternalMotion;
            controlPanel=FindFirstObjectByType<RobotControlPanel>();
            if(controlPanel){priorHitTest=controlPanel.AdditionalUiHitTest;hitTest=p=>(panelOpen&&panel.Contains(new Vector2(p.x,Screen.height-p.y)))||(priorHitTest!=null&&priorHitTest(p));controlPanel.AdditionalUiHitTest=hitTest;}
        }
        public void TogglePanel(){panelOpen=!panelOpen;}
        public void StartCamera()
        {
            StopCamera(); panelOpen=true;
            try
            {
                var directory=new DirectoryInfo(Application.dataPath);
                string root=null;
                for(int i=0;i<5 && directory!=null;i++,directory=directory.Parent)
                    if(File.Exists(Path.Combine(directory.FullName,"CameraTracking","tracker.py")))
                    {root=Path.Combine(directory.FullName,"CameraTracking");break;}
                if(root==null)throw new IOException("CameraTracking folder was not found beside this Unity project.");
                string python=Path.Combine(root,".venv","Scripts","pythonw.exe");
                if(!File.Exists(python))throw new IOException("Camera runtime is missing. See CameraTracking/README.md.");
                var probe=new TcpListener(IPAddress.Loopback,0);probe.Start();int port=((IPEndPoint)probe.LocalEndpoint).Port;probe.Stop();
                token=Guid.NewGuid().ToString("N");url="http://127.0.0.1:"+port+"/frame";
                helper=System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo(python,
                    "\""+Path.Combine(root,"tracker.py")+"\" --camera "+cameraIndex+" --port "+port+" --token "+token)
                    {UseShellExecute=false,CreateNoWindow=true,WorkingDirectory=root});
                Status="Opening camera and loading hand tracking…";
                polling=StartCoroutine(Poll());
            }
            catch(Exception e){Status=e.Message;StopHelper();}
        }
        private IEnumerator Poll()
        {
            float started=Time.realtimeSinceStartup;
            while(helper!=null)
            {
                if(helper.HasExited){Pause("Camera stopped. Start camera to retry.");break;}
                using(var next=UnityWebRequest.Get(url))
                {
                    request=next;next.timeout=2;next.SetRequestHeader("X-Tracking-Token",token);
                    yield return next.SendWebRequest();
                    request=null;
                    if(next.result==UnityWebRequest.Result.Success)
                    {
                        CameraLandmarks incoming=null;
                        try{incoming=JsonUtility.FromJson<CameraLandmarks>(next.downloadHandler.text);}catch(Exception){ }
                        if(incoming!=null && incoming.frame>lastFrame)
                        {
                            if(incoming.ready && incoming.version!=2)
                            {Pause("Camera data version mismatch. Restart camera and use the updated simulator.");sample=null;continue;}
                            lastFrame=incoming.frame;lastFresh=Time.realtimeSinceStartup;
                            sample=incoming;
                            if(!string.IsNullOrEmpty(incoming.jpeg))
                            {
                                try{if(!preview)preview=new Texture2D(2,2);preview.LoadImage(Convert.FromBase64String(incoming.jpeg));}catch(Exception){ }
                                incoming.jpeg=null;
                            }
                            if(!Following)Status=(string.IsNullOrEmpty(pauseReason)?"":pauseReason+"\n")+
                                (incoming.Usable?"Palm found. Center & follow when ready.":"Show ONE open palm facing the camera.");
                        }
                        else if(incoming!=null && !incoming.ready)Status=incoming.message;
                    }
                    else if(Time.realtimeSinceStartup-started>15)Status="Waiting for camera. Try another camera index if this continues.";
                }
                yield return new WaitForSecondsRealtime(.04f);
            }
        }
        public bool BeginFollowing()
        {
            if(HardwareBlocked){Status="Disconnect Robot link before using camera simulation.";return false;}
            if(!HasFreshTracking){Status="Show one open palm facing the camera before starting.";return false;}
            // Stop existing automatic modes before taking ownership; no physical transport calls.
            arm.Stop();
            center=new Vector2(sample.hx,sample.hy);origin=arm.EndEffectorMillimeters;handLabel=sample.label;
            referencePalm=sample.palm;
            pauseReason=null;
            smoothHand=CameraTrackingMath.HandTarget(sample,center,origin,spanMillimeters,referencePalm,depthGain);
            Following=true;nextMove=0;MakeMarkers();
            Status="Following and pointing toward your hand.";
            return true;
        }
        public void Pause(string reason="Camera follow paused. Center & follow to resume.")
        {
            bool wasFollowing=Following;Following=false;Status=reason;pauseReason=wasFollowing?reason:null;
            if(wasFollowing && arm){ownCommand=true;try{arm.Stop();}finally{ownCommand=false;}}
            SetMarkers(false);
        }
        private void ExternalMotion()
        {
            if(ownCommand || !Following)return;
            Following=false;Status="Paused by another control. Center & follow to resume.";pauseReason=Status;SetMarkers(false);
        }
        private void Update()
        {
            if(!Following)return;
            if(HardwareBlocked){Pause("Camera follow stopped: Robot link connected.");return;}
            var controller=WindowsXboxInput.Read();
            if(controller.connected&&!controller.Neutral){Pause("Paused by controller input. Release controls to take over.");return;}
            if(!HasFreshTracking || sample.label!=handLabel){Pause("Palm lost, turned sideways, or hand changed. Show your palm, then Center & follow.");return;}
            if(Input.GetKeyDown(KeyCode.Escape)){Pause();return;}
            if(Time.realtimeSinceStartup<nextMove)return;
            nextMove=Time.realtimeSinceStartup+.04f;
            Vector3 hand=CameraTrackingMath.HandTarget(sample,center,origin,spanMillimeters,referencePalm,depthGain);
            smoothHand=Vector3.Lerp(smoothHand,hand,.3f);
            var handWorld=World(smoothHand);var toolWorld=World(CameraTrackingMath.ToolTarget(smoothHand,origin));
            var snapshot=ForwardKinematics.Capture(arm);
            Vector3 forward=GripperForward(arm);
            var pose=CameraTrackingMath.Step(snapshot,forward,toolWorld,handWorld,1.8f,aimHand);
            // Recheck immediately before issuing a target; camera never invokes ARM or a transport.
            if(HardwareBlocked){Pause("Robot link connected; camera simulation stopped.");return;}
            ownCommand=true;try{arm.MoveToPose(pose);}finally{ownCommand=false;}
            handMarker.position=handWorld;toolMarker.position=toolWorld;
            aimLine.SetPosition(0,arm.endEffector.position);aimLine.SetPosition(1,arm.endEffector.position+forward*.12f);
            float error=Vector3.Distance(arm.endEffector.position,toolWorld)*snapshot.millimetersPerWorldUnit;
            float angle=Vector3.Angle(forward,handWorld-arm.endEffector.position);
            Status="SIMULATION FOLLOW  |  Target error "+error.ToString("0")+" mm\nHand aim error "+angle.ToString("0")+"°"+
                (angle>20?" — wrist / reach limited":"")+"\nDepth offset (estimated) "+(smoothHand.z-origin.z-160).ToString("+0;-0;0")+" mm";
        }
        public static Vector3 GripperForward(RobotArm robot)
        {
            var linkage=robot.GetComponent<GripperLinkage>();
            if(linkage && linkage.sides!=null && linkage.sides.Length==2)
                return (robot.endEffector.position-linkage.wristFrame.TransformPoint((linkage.sides[0].a+linkage.sides[1].a)*.5f)).normalized;
            return (robot.endEffector.position-robot.joints[3].transform.position).normalized;
        }
        private Vector3 World(Vector3 mm)=>arm.robotOrigin.TransformPoint(mm/arm.millimetersPerUnityUnit);
        private void MakeMarkers()
        {
            if(!handMarker)handMarker=Marker("Camera virtual hand",new Color(.1f,.8f,1),.025f);
            if(!toolMarker)toolMarker=Marker("Camera gripper goal",new Color(1,.7f,.05f),.014f);
            if(!aimLine)
            {
                aimLine=new GameObject("Gripper pointing ray").AddComponent<LineRenderer>();
                aimLine.material=new Material(Shader.Find("Sprites/Default"));aimLine.startColor=aimLine.endColor=Color.green;
                aimLine.startWidth=aimLine.endWidth=.002f;aimLine.positionCount=2;
            }
            SetMarkers(true);
        }
        private Transform Marker(string name,Color color,float diameter)
        {
            var obj=GameObject.CreatePrimitive(PrimitiveType.Sphere);obj.name=name;Remove(obj.GetComponent<Collider>());
            obj.transform.localScale=Vector3.one*diameter;obj.GetComponent<Renderer>().material.color=color;return obj.transform;
        }
        private void SetMarkers(bool visible)
        {if(handMarker)handMarker.gameObject.SetActive(visible);if(toolMarker)toolMarker.gameObject.SetActive(visible);if(aimLine)aimLine.gameObject.SetActive(visible);}
        public void StopCamera()
        {
            Pause("Camera stopped.");if(polling!=null){StopCoroutine(polling);polling=null;}
            if(request!=null){request.Abort();request.Dispose();request=null;}StopHelper();
            sample=null;lastFrame=0;if(preview){Remove(preview);preview=null;}
        }
        private void StopHelper()
        {
            if(helper==null)return;
            try
            {
                if(!helper.HasExited)
                {
                    try
                    {
                        var stop=(HttpWebRequest)WebRequest.Create(url.Replace("/frame","/stop"));
                        stop.Method="POST";stop.ContentLength=0;stop.Timeout=700;stop.ReadWriteTimeout=700;
                        stop.Headers["X-Tracking-Token"]=token;using(var response=stop.GetResponse()){}
                    }
                    catch(Exception){}
                    if(!helper.WaitForExit(1000))
                    {
                        // Windows venv launches a child Python process; close only this owned tree.
                        using(var kill=System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo(
                            Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.System),"taskkill.exe"),
                            "/PID "+helper.Id+" /T /F"){UseShellExecute=false,CreateNoWindow=true}))kill?.WaitForExit(1000);
                    }
                }
                helper.Dispose();
            }
            catch(Exception){}
            helper=null;
        }
        private void OnApplicationFocus(bool focus){if(!focus && Following)Pause("Paused because the app lost focus. Center & follow to resume.");}
        private void OnApplicationQuit(){StopCamera();}
        private void OnDisable(){if(Following)Pause();}
        private void OnDestroy()
        {
            if(arm){arm.MotionCommandIssued-=ExternalMotion;arm.SimulationStopped-=ExternalMotion;}
            if(controlPanel&&controlPanel.AdditionalUiHitTest==hitTest)controlPanel.AdditionalUiHitTest=priorHitTest;
            StopCamera();if(handMarker)Remove(handMarker.gameObject);if(toolMarker)Remove(toolMarker.gameObject);if(aimLine)Remove(aimLine.gameObject);
        }
        private static void Remove(UnityEngine.Object value){if(Application.isPlaying)Destroy(value);else DestroyImmediate(value);}
        private void OnGUI()
        {
            if(!panelOpen)return;
            if(wrap==null)wrap=new GUIStyle(GUI.skin.label){wordWrap=true};
            panel.height=Mathf.Min(655,Screen.height-135);panel=GUI.Window(840731,panel,DrawPanel,"CAMERA TRACKING · simulation");
        }
        private void DrawPanel(int id)
        {
            panelScroll=GUILayout.BeginScrollView(panelScroll);
            GUILayout.Label("Gripper points toward your hand. Move farther away to reach; closer to pull back.",wrap);
            if(preview)GUILayout.Label(preview,GUILayout.Width(310),GUILayout.Height(232));
            else GUILayout.Box("Camera preview",GUILayout.Width(310),GUILayout.Height(120));
            GUILayout.Label("Hand only · Mirrored preview · No face tracking",wrap);
            GUILayout.BeginHorizontal();GUILayout.Label("Camera "+cameraIndex);
            GUI.enabled=helper==null;
            if(GUILayout.Button("−",GUILayout.Width(34)))cameraIndex=Mathf.Max(0,cameraIndex-1);
            if(GUILayout.Button("+",GUILayout.Width(34)))cameraIndex=Mathf.Min(9,cameraIndex+1);
            GUI.enabled=true;
            if(GUILayout.Button(helper==null?"Start camera":"Stop camera")){if(helper==null)StartCamera();else StopCamera();}
            GUILayout.EndHorizontal();
            aimHand=GUILayout.Toggle(aimHand,"Point gripper toward hand");
            GUILayout.Label("Hand movement scale: "+spanMillimeters.ToString("0")+" mm / image width");
            spanMillimeters=GUILayout.HorizontalSlider(spanMillimeters,150,600);
            GUILayout.Label("Depth response: "+depthGain.ToString("0")+" (estimated from palm size)");
            depthGain=GUILayout.HorizontalSlider(depthGain,50,300);
            GUI.enabled=HasFreshTracking&&!HardwareBlocked;
            if(GUILayout.Button(Following?"Recenter hand here":"Center & follow hand",GUILayout.Height(32)))BeginFollowing();
            GUI.enabled=true;
            if(GUILayout.Button("STOP FOLLOW",GUILayout.Height(29)))Pause();
            GUILayout.Label(Status,wrap);
            if(HardwareBlocked)GUILayout.Label("Disconnect Robot link to try this in simulation.",wrap);
            GUILayout.Label("Keep your palm facing the camera. Smaller palm = farther / reach; larger = closer / retract. Depth is approximate. Turning your hand affects it. Keep the camera fixed for this mode. Tracking loss pauses; click Center & follow to restart.",wrap);
            if(GUILayout.Button("Hide panel"))panelOpen=false;
            GUILayout.EndScrollView();
            GUI.DragWindow(new Rect(0,0,10000,22));
        }
    }
}
