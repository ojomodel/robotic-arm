using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;

namespace CareerFair.Robot
{
    [Serializable,DataContract] public sealed class PickAndPlaceWaypoint
    {
        [DataMember(IsRequired=true)] public string name;
        [DataMember(IsRequired=true)] public float[] angles;
        [DataMember(IsRequired=true)] public float dwellSeconds;
    }
    [Serializable,DataContract] public sealed class PickAndPlaceProgram
    {
        [DataMember(IsRequired=true)] public int version=1;
        [DataMember(IsRequired=true)] public List<PickAndPlaceWaypoint> waypoints=new List<PickAndPlaceWaypoint>();
    }
    /// <summary>Ordered model-command waypoints only; contains no transport, calibration or automatic start.</summary>
    public static class PickAndPlaceProgramStore
    {
        public static bool Validate(PickAndPlaceProgram program,out string reason)
        {
            if(program==null || program.version!=1 || program.waypoints==null || program.waypoints.Count>32)
                return Fail("A saved program supports up to 32 recordings in version 1.",out reason);
            var names=new HashSet<string>(StringComparer.Ordinal);
            foreach(var step in program.waypoints)
                if(step==null || string.IsNullOrWhiteSpace(step.name) || step.name.Length>64 || !names.Add(step.name) ||
                    step.angles==null || step.angles.Length!=5 || step.angles.Any(v=>!Finite(v)) ||
                    !Finite(step.dwellSeconds) || step.dwellSeconds<0 || step.dwellSeconds>30)
                    return Fail("Each waypoint needs a unique name, five finite angles and a pause from0 to30 seconds.",out reason);
            reason=null; return true;
        }
        public static PickAndPlaceProgram DeepClone(PickAndPlaceProgram program)
        {
            if(program==null) return new PickAndPlaceProgram();
            return new PickAndPlaceProgram { version=program.version,waypoints=program.waypoints==null ? null : program.waypoints.Select(s=>s==null ? null :
                new PickAndPlaceWaypoint {name=s.name,angles=s.angles==null ? null : (float[])s.angles.Clone(),dwellSeconds=s.dwellSeconds}).ToList() };
        }
        public static bool Load(string path,out PickAndPlaceProgram program,out string reason)
        {
            program=null;
            try
            {
                using(var stream=new FileStream(path,FileMode.Open,FileAccess.Read,FileShare.Read))
                {
                    if(stream.Length>65536) return Fail("Saved program is too large.",out reason);
                    var candidate=(PickAndPlaceProgram)new DataContractJsonSerializer(typeof(PickAndPlaceProgram)).ReadObject(stream);
                    if(!Validate(candidate,out reason)) return false;
                    program=DeepClone(candidate); return true;
                }
            }
            catch(Exception e) when(Recoverable(e)) { return Fail("Could not load program: "+e.Message,out reason); }
        }
        public static bool Save(string path,PickAndPlaceProgram program,out string reason)
        {
            if(!Validate(program,out reason)) return false;
            string temporary=null;
            try
            {
                string full=Path.GetFullPath(path),directory=Path.GetDirectoryName(full);
                Directory.CreateDirectory(directory);
                temporary=Path.Combine(directory,"."+Path.GetFileName(full)+"."+Guid.NewGuid().ToString("N")+".tmp");
                using(var stream=new FileStream(temporary,FileMode.CreateNew,FileAccess.Write,FileShare.None))
                { new DataContractJsonSerializer(typeof(PickAndPlaceProgram)).WriteObject(stream,DeepClone(program)); stream.Flush(true); }
                if(File.Exists(full)) File.Replace(temporary,full,null); else File.Move(temporary,full);
                temporary=null; reason=null; return true;
            }
            catch(Exception e) when(Recoverable(e)) { return Fail("Could not save program: "+e.Message,out reason); }
            finally { if(temporary!=null) try {File.Delete(temporary);} catch(Exception e) when(Recoverable(e)) {} }
        }
        private static bool Finite(float value)=>!float.IsNaN(value)&&!float.IsInfinity(value);
        private static bool Fail(string message,out string reason){reason=message;return false;}
        private static bool Recoverable(Exception e)=>e is IOException||e is UnauthorizedAccessException||e is ArgumentException||
            e is NotSupportedException||e is SerializationException||e is System.Xml.XmlException||e is System.Security.SecurityException;
    }
}
