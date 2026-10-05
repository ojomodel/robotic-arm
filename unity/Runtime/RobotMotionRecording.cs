using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;

namespace CareerFair.Robot
{
    [Serializable,DataContract] public sealed class RobotMotionSample
    {
        [DataMember(IsRequired=true)] public float timeSeconds;
        [DataMember(IsRequired=true)] public float[] angles;
    }
    [Serializable,DataContract] public sealed class RobotMotionRecording
    {
        [DataMember(IsRequired=true)] public int version=1;
        [DataMember(IsRequired=true)] public string id=Guid.NewGuid().ToString("N");
        [DataMember(IsRequired=true)] public string name="Recording";
        [DataMember(IsRequired=true)] public string savedUtc=DateTime.UtcNow.ToString("O",CultureInfo.InvariantCulture);
        [DataMember(IsRequired=true)] public List<RobotMotionSample> samples=new List<RobotMotionSample>();
    }
    public sealed class RobotMotionRecordingInfo
    {
        public string id,name,savedUtc,path;
        public float durationSeconds;
        public int sampleCount;
    }
    /// <summary>Pure storage for separate, time-sampled recordings; never interprets waypoint programs or sends commands.</summary>
    public static class RobotMotionRecordingStore
    {
        public const float MaximumDurationSeconds=600f;
        public const int MaximumSamples=12002;
        public const long MaximumFileBytes=8L*1024*1024;

        public static bool Validate(RobotMotionRecording clip,out string reason)
        {
            if(clip==null||clip.version!=1)return Fail("Recording version is missing or unsupported.",out reason);
            if(!Guid.TryParse(clip.id,out _))return Fail("Recording ID must be a GUID.",out reason);
            if(string.IsNullOrWhiteSpace(clip.name)||clip.name.Length>64)return Fail("Give the recording a name of1–64 characters.",out reason);
            if(string.IsNullOrEmpty(clip.savedUtc)||clip.savedUtc.Length>64||
                !DateTime.TryParse(clip.savedUtc,CultureInfo.InvariantCulture,DateTimeStyles.RoundtripKind,out var saved)||saved.Kind!=DateTimeKind.Utc)
                return Fail("Recording date must be a UTC timestamp.",out reason);
            if(clip.samples==null||clip.samples.Count<2||clip.samples.Count>MaximumSamples)
                return Fail("A recording needs2–12002 samples.",out reason);
            float previous=-1;
            for(int i=0;i<clip.samples.Count;i++)
            {
                var sample=clip.samples[i];
                if(sample==null||!Finite(sample.timeSeconds)||sample.timeSeconds<0||sample.timeSeconds>MaximumDurationSeconds||
                    (i==0?sample.timeSeconds!=0:sample.timeSeconds<=previous))
                    return Fail("Recording timestamps must start at0, strictly increase, and end within600 seconds.",out reason);
                if(sample.angles==null||sample.angles.Length!=5||sample.angles.Any(v=>!Finite(v)))
                    return Fail("Every recording sample needs five finite joint commands.",out reason);
                previous=sample.timeSeconds;
            }
            reason=null;return true;
        }
        public static RobotMotionRecording DeepClone(RobotMotionRecording clip)
        {
            if(clip==null)return null;
            return new RobotMotionRecording{version=clip.version,id=clip.id,name=clip.name,savedUtc=clip.savedUtc,
                samples=clip.samples==null?null:clip.samples.Select(s=>s==null?null:new RobotMotionSample{
                    timeSeconds=s.timeSeconds,angles=s.angles==null?null:(float[])s.angles.Clone()}).ToList()};
        }
        public static bool Save(string directory,RobotMotionRecording clip,out string reason)
        {
            if(!Validate(clip,out reason))return false;
            string temporary=null;
            try
            {
                if(string.IsNullOrWhiteSpace(directory))return Fail("A recording directory is required.",out reason);
                string folder=Path.GetFullPath(directory);Directory.CreateDirectory(folder);
                // Only a canonical GUID determines a filename. The display name is never a path.
                string path=Path.Combine(folder,Guid.Parse(clip.id).ToString("N")+".json");
                temporary=Path.Combine(folder,"."+Guid.NewGuid().ToString("N")+".tmp");
                using(var stream=new FileStream(temporary,FileMode.CreateNew,FileAccess.Write,FileShare.None))
                {
                    new DataContractJsonSerializer(typeof(RobotMotionRecording)).WriteObject(stream,DeepClone(clip));
                    if(stream.Length>MaximumFileBytes)return Fail("Recording exceeds the8MB file limit.",out reason);
                    stream.Flush(true);
                }
                if(File.Exists(path))File.Replace(temporary,path,null);else File.Move(temporary,path);
                temporary=null;reason=null;return true;
            }
            catch(Exception e)when(Recoverable(e)){return Fail("Could not save recording: "+e.Message,out reason);}
            finally{if(temporary!=null)try{File.Delete(temporary);}catch(Exception e)when(Recoverable(e)){} }
        }
        public static bool Load(string path,out RobotMotionRecording clip,out string reason)
        {
            clip=null;
            try
            {
                using(var stream=new FileStream(path,FileMode.Open,FileAccess.Read,FileShare.Read))
                {
                    if(stream.Length>MaximumFileBytes)return Fail("Recording exceeds the8MB file limit.",out reason);
                    var candidate=(RobotMotionRecording)new DataContractJsonSerializer(typeof(RobotMotionRecording)).ReadObject(stream);
                    if(!Validate(candidate,out reason))return false;
                    clip=DeepClone(candidate);return true;
                }
            }
            catch(Exception e)when(Recoverable(e)){return Fail("Could not load recording: "+e.Message,out reason);}
        }
        public static List<RobotMotionRecordingInfo> List(string directory,out string reason)
        {
            var result=new List<RobotMotionRecordingInfo>();reason=null;
            try
            {
                if(string.IsNullOrWhiteSpace(directory)){reason="A recording directory is required.";return result;}
                if(!Directory.Exists(directory))return result;
                int skipped=0;
                foreach(string path in Directory.GetFiles(directory,"*.json"))
                {
                    if(!Load(path,out var clip,out _)){skipped++;continue;}
                    result.Add(new RobotMotionRecordingInfo{id=clip.id,name=clip.name,savedUtc=clip.savedUtc,path=Path.GetFullPath(path),
                        durationSeconds=clip.samples[clip.samples.Count-1].timeSeconds,sampleCount=clip.samples.Count});
                }
                if(skipped>0)reason="Skipped "+skipped+" invalid recording file(s); existing files were not changed.";
                return result.OrderByDescending(c=>c.savedUtc,StringComparer.Ordinal).ThenBy(c=>c.name,StringComparer.Ordinal).ThenBy(c=>c.id,StringComparer.Ordinal).ToList();
            }
            catch(Exception e)when(Recoverable(e)){reason="Could not list recordings: "+e.Message;return result;}
        }
        private static bool Finite(float value)=>!float.IsNaN(value)&&!float.IsInfinity(value);
        private static bool Fail(string message,out string reason){reason=message;return false;}
        private static bool Recoverable(Exception e)=>e is IOException||e is UnauthorizedAccessException||e is ArgumentException||
            e is NotSupportedException||e is SerializationException||e is System.Xml.XmlException||e is System.Security.SecurityException;
    }
}
