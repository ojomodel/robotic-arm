using System;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;

namespace CareerFair.Robot.Hardware
{
    /// <summary>Independent operator speed preference. Does not load calibration, Unity state or hardware.</summary>
    public static class RobotSpeedSettings
    {
        public const float Minimum = 1f;
        public const float Maximum = 400f;
        public const float Default = 100f;
        private const int CurrentVersion = 1;
        private const long MaximumFileBytes = 4096;

        public sealed class Result
        {
            public float Speed { get; private set; }
            public bool Success { get; private set; }
            public bool UsedDefault { get; private set; }
            public bool WasClamped { get; private set; }
            public string Message { get; private set; }
            internal Result(float speed, bool success, bool usedDefault, bool wasClamped, string message)
            { Speed=speed; Success=success; UsedDefault=usedDefault; WasClamped=wasClamped; Message=message; }
        }

        [DataContract] private sealed class Document
        {
            [DataMember(Name="version", IsRequired=true)] public int Version;
            [DataMember(Name="degreesPerSecond", IsRequired=true)] public float Speed;
        }

        public static float Clamp(float value) => Finite(value) ? Math.Max(Minimum,Math.Min(Maximum,value)) : Default;

        public static Result Load(string path)
        {
            try
            {
                string fullPath = ValidatePath(path);
                if(Directory.Exists(fullPath)) return Failure("Robot speed settings path is a directory.");
                Document document;
                using(var stream=new FileStream(fullPath,FileMode.Open,FileAccess.Read,FileShare.Read))
                {
                    if(stream.Length>MaximumFileBytes) return Failure("Robot speed settings file is too large.");
                    document=(Document)new DataContractJsonSerializer(typeof(Document)).ReadObject(stream);
                }
                if(document==null || document.Version!=CurrentVersion)
                    return Failure("Robot speed settings version is missing or unsupported.");
                if(!Finite(document.Speed)) return Failure("Saved robot speed is not finite.");
                float speed=Clamp(document.Speed); bool clamped=speed!=document.Speed;
                return new Result(speed,true,false,clamped,clamped ? "Saved robot speed was clamped to the supported 1–400 degrees/s range." : null);
            }
            catch(FileNotFoundException) { return Missing(); }
            catch(DirectoryNotFoundException) { return Missing(); }
            catch(Exception e) when(Recoverable(e)) { return Failure("Could not load robot speed: " + e.Message); }
        }

        public static Result Save(string path, float value)
        {
            if(!Finite(value)) return Failure("Robot speed must be finite; the saved speed was not changed.");
            string temporaryPath=null;
            try
            {
                string fullPath=ValidatePath(path);
                string directory=Path.GetDirectoryName(fullPath);
                Directory.CreateDirectory(directory);
                float speed=Clamp(value); bool clamped=speed!=value;
                temporaryPath=Path.Combine(directory,"."+Path.GetFileName(fullPath)+"."+Guid.NewGuid().ToString("N")+".tmp");
                using(var stream=new FileStream(temporaryPath,FileMode.CreateNew,FileAccess.Write,FileShare.None))
                {
                    new DataContractJsonSerializer(typeof(Document)).WriteObject(stream,new Document { Version=CurrentVersion,Speed=speed });
                    stream.Flush(true);
                }
                // The temporary file is on the same volume. Replace never deletes the
                // last good file first; unsupported/failed atomic commits are reported.
                if(File.Exists(fullPath)) File.Replace(temporaryPath,fullPath,null);
                else File.Move(temporaryPath,fullPath);
                temporaryPath=null;
                return new Result(speed,true,false,clamped,clamped ? "Robot speed was clamped and saved within 1–400 degrees/s." : null);
            }
            catch(Exception e) when(Recoverable(e)) { return Failure("Could not save robot speed: " + e.Message); }
            finally
            {
                if(temporaryPath!=null)
                {
                    try { File.Delete(temporaryPath); }
                    catch(Exception e) when(Recoverable(e)) { /* The primary save failure remains visible. */ }
                }
            }
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static string ValidatePath(string path)
        {
            if(string.IsNullOrWhiteSpace(path)) throw new ArgumentException("A robot speed settings path is required.",nameof(path));
            return Path.GetFullPath(path);
        }
        private static bool Recoverable(Exception e) => e is IOException || e is UnauthorizedAccessException ||
            e is ArgumentException || e is NotSupportedException || e is SerializationException ||
            e is System.Xml.XmlException || e is System.Security.SecurityException;
        private static Result Missing() => new Result(Default,true,true,false,"No saved robot speed; requesting 100 degrees/s, subject to the connected firmware limit.");
        private static Result Failure(string message) => new Result(Default,false,true,false,message);
    }
}
