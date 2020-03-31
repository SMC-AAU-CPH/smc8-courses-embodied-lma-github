using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Windows.Kinect;
using Joint = Windows.Kinect.Joint;

public class LabanDescriptors : MonoBehaviour
{
    // Vectors
    Vector3 faceNormal;   //Normal vector of the plane of head/shoulders
    Vector3 faceLeft;     //Vector from face to left shoulder needed to describe the plane
    Vector3 faceRight;    //Same as above, but for right
    Vector3 root;         //Root joint


    //Enum defining types of efforts
    private enum Efforts { Weight, Space, Time, Flow };

    //Public Accessors for Effort Values. Currently only returns for one body and needs to be refactored to allow more.
    public float WeightEffort { get; private set; }
    public float TimeEffort { get; private set; }
    public float SpaceEffort { get; private set; }
    public float FlowEffort { get; private set; }
    public Vector3 RootDirection { get; private set; }

    //Kinect Dependencies
    public GameObject BodySourceManager;
    private BodySourceManager _BodyManager;

    public GameObject jointPrefab;

    // All Bodies tracked by the Kinect
    private Dictionary<ulong, GameObject> _Bodies = new Dictionary<ulong, GameObject>();

    //List of joints that will be used to measure efforts - can be altered to just get individual limbs or joints.

    private List<JointType> _Joints = new List<JointType>
    {
        JointType.FootLeft,
        JointType.AnkleLeft,
        JointType.KneeLeft, 
        JointType.HipLeft,

        JointType.FootRight, 
        JointType.AnkleRight, 
        JointType.KneeRight,
        JointType.HipRight,

        JointType.HandTipLeft,
        JointType.ThumbLeft,
        JointType.HandLeft,
        JointType.WristLeft,
        JointType.ElbowLeft,
        JointType.ShoulderLeft,

        JointType.HandTipRight, 
        JointType.ThumbRight, 
        JointType.HandRight, 
        JointType.WristRight,
        JointType.ElbowRight,
        JointType.ShoulderRight, 
        
        JointType.SpineBase,
        JointType.SpineMid,
        JointType.SpineShoulder, 
        JointType.Neck, 
        JointType.Head
    };

    // List of joints used for each specific weight
    private Dictionary<Efforts, List<JointType>> effortJoints = new Dictionary<Efforts, List<JointType>>();
    private List<JointType> weightJoints = new List<JointType>
        {
            JointType.SpineBase,
            JointType.FootLeft,
            JointType.FootRight,
            JointType.HandTipLeft,
            JointType.HandTipRight
        };
    private List<JointType> timeJoints = new List<JointType>
        {
            JointType.SpineBase,
            JointType.FootLeft,
            JointType.FootRight,
            JointType.HandTipLeft,
            JointType.HandTipRight
        };
    private List<JointType> spaceJoints = new List<JointType>
        {
            JointType.SpineBase,
            JointType.Head,
            JointType.ShoulderLeft,
            JointType.ShoulderRight
        };
    private List<JointType> flowJoints = new List<JointType>
        {
        //doesn't specify which joints to use for flow calculations
            JointType.SpineBase,
            JointType.FootLeft,
            JointType.FootRight,
            JointType.HandTipLeft,
            JointType.HandTipRight
        };

    // Transforms (Position & Rotation), Velocity, and Previous Positions (used for velocity calculations) of each Joint

    private Dictionary<ulong, Dictionary<JointType, Transform>> BodyJoints = new Dictionary<ulong, Dictionary<JointType, Transform>>();
    private Dictionary<ulong, Dictionary<JointType, Vector3>> BodyJoints_Velocity = new Dictionary<ulong, Dictionary<JointType, Vector3>>();
    private Dictionary<ulong, Dictionary<JointType, Vector3>> BodyJoints_PrevPos = new Dictionary<ulong, Dictionary<JointType, Vector3>>();
    private Dictionary<ulong, Dictionary<JointType, Vector3>> BodyJoints_Acceleration = new Dictionary<ulong, Dictionary<JointType, Vector3>>();
    private Dictionary<ulong, Dictionary<JointType, Vector3>> BodyJoints_Jerk = new Dictionary<ulong, Dictionary<JointType, Vector3>>();

    // Frame by Frame values for each Effort per tracked body.

    private Dictionary<ulong, List<float>> WeightEffortRealTime = new Dictionary<ulong, List<float>>();
    private Dictionary<ulong, List<float>> TimeEffortRealTime = new Dictionary<ulong, List<float>>();
    private Dictionary<ulong, List<float>> SpaceEffortRealTime = new Dictionary<ulong, List<float>>();
    private Dictionary<ulong, List<float>> FlowEffortRealTime = new Dictionary<ulong, List<float>>();

    // Final Effort values for each tracked body
    public Dictionary<ulong, float> BodyWeightEffort = new Dictionary<ulong, float>();
    public Dictionary<ulong, float> BodyTimeEffort = new Dictionary<ulong, float>();
    public Dictionary<ulong, float> BodySpaceEffort = new Dictionary<ulong, float>();
    public Dictionary<ulong, float> BodyFlowEffort = new Dictionary<ulong, float>();



    //Triggers for running Calculation Coroutines at distinct time intervals
    private Dictionary<Efforts, bool> effortCalculationsRunning = new Dictionary<Efforts, bool> {
        [Efforts.Weight] = false,
        [Efforts.Flow] = false,
        [Efforts.Time] = false,
        [Efforts.Space] = false,
    };

    private void Start()
    {
        effortJoints.Add(Efforts.Weight, weightJoints);
        effortJoints.Add(Efforts.Time, timeJoints);
        effortJoints.Add(Efforts.Space, spaceJoints);
        effortJoints.Add(Efforts.Flow, flowJoints);
    }

    void Update()
    {
        #region Get Kinect Data
        if (BodySourceManager == null)
        {
            return;
        }

        _BodyManager = BodySourceManager.GetComponent<BodySourceManager>();
        if (_BodyManager == null)
        {
            return;
        }

        Body[] data = _BodyManager.GetData();
        if (data == null)
        {
            return;
        }

        List<ulong> trackedIds = new List<ulong>();
        foreach (var body in data)
        {
            if (body == null)
            {
                continue;
            }

            if (body.IsTracked)
            {
                trackedIds.Add(body.TrackingId);
            }
        }
        #endregion

        #region Delete Kinect Bodies
        List<ulong> knownIds = new List<ulong>(_Bodies.Keys);

        // First delete untracked bodies
        foreach (ulong trackingId in knownIds)
        {
            if (!trackedIds.Contains(trackingId))
            {
                Destroy(_Bodies[trackingId]);
                _Bodies.Remove(trackingId);
            }
        }
        #endregion

        #region Create Kinect Bodies
        foreach (var body in data)
        {
            if (body == null)
            {
                continue;
            }

            if (body.IsTracked)
            {
                if (!_Bodies.ContainsKey(body.TrackingId))
                {
                    _Bodies[body.TrackingId] = CreateBodyObject(body.TrackingId);
                }

                UpdateBodyObject(body, body.TrackingId);
            }
        }
        #endregion

        #region Laban Descriptors
        foreach (ulong id in trackedIds)
        {
            WeightEffortRealTime[id].Add(CalculateEffort(id, Efforts.Weight));
            SpaceEffortRealTime[id].Add(CalculateEffort(id, Efforts.Space));
            TimeEffortRealTime[id].Add(CalculateEffort(id, Efforts.Time));
            FlowEffortRealTime[id].Add(CalculateEffort(id, Efforts.Flow));

            if (!effortCalculationsRunning[Efforts.Weight])
                StartCoroutine(CalculateFinalEffort(id, WeightEffortRealTime[id], 0.5f, Efforts.Weight));

            if (!effortCalculationsRunning[Efforts.Time])
                StartCoroutine(CalculateFinalEffort(id, TimeEffortRealTime[id], 0.5f, Efforts.Time));
      
            if (!effortCalculationsRunning[Efforts.Flow])
                StartCoroutine(CalculateFinalEffort(id, FlowEffortRealTime[id], 0.5f, Efforts.Flow));

            if (!effortCalculationsRunning[Efforts.Space])
                StartCoroutine(CalculateFinalEffort(id, SpaceEffortRealTime[id], 0.5f, Efforts.Space));
            

            //FIXME
            // UI ONLY WORKS WITH ONE BODY
            if (BodyWeightEffort[id] > 0)
                WeightEffort = BodyWeightEffort[id];

            //if (BodyTimeEffort[id] > 0)
                TimeEffort = BodyTimeEffort[id];

                SpaceEffort = BodySpaceEffort[id];

            if (BodyFlowEffort[id] > 0)
                FlowEffort = BodyFlowEffort[id];
            RootDirection = BodyJoints_Velocity[id][JointType.SpineBase].normalized;
        }
        #endregion

    }

    private GameObject CreateBodyObject(ulong id)
    {
        GameObject body = new GameObject("body_" + id);
        body.tag = "Body";
        Dictionary<JointType, Transform> JointTransforms = new Dictionary<JointType, Transform>();
        Dictionary<JointType, Vector3> JointVelocity = new Dictionary<JointType, Vector3>();
        Dictionary<JointType, Vector3> JointPrevPos = new Dictionary<JointType, Vector3>();
        Dictionary<JointType, Vector3> JointAcceleraion = new Dictionary<JointType, Vector3>();
        Dictionary<JointType, Vector3> JointJerk = new Dictionary<JointType, Vector3>();

        foreach (JointType joint in _Joints)
        {
            GameObject jointObj = Instantiate(jointPrefab, transform);//GameObject.CreatePrimitive(PrimitiveType.Cube); //put body prefabs here???
            jointObj.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);
            jointObj.name = joint.ToString();
            jointObj.transform.parent = body.transform;
            //Initialize Joint Dictionaries
            JointTransforms.Add(joint, jointObj.transform);
            JointVelocity.Add(joint, Vector3.zero);
            JointPrevPos.Add(joint, jointObj.transform.position);
        }
        //Initialize whole body Dictionaries
        BodyJoints.Add(id, JointTransforms);
        BodyJoints_Velocity.Add(id, JointVelocity);
        BodyJoints_PrevPos.Add(id, JointPrevPos);
        BodyJoints_Acceleration.Add(id, JointAcceleraion);
        BodyJoints_Jerk.Add(id, JointJerk);

        BodyWeightEffort[id] = 0;
        BodySpaceEffort[id] = 0;
        BodyTimeEffort[id] = 0;
        BodyFlowEffort[id] = 0;

        WeightEffortRealTime.Add(id, new List<float>());
        SpaceEffortRealTime.Add(id, new List<float>());
        TimeEffortRealTime.Add(id, new List<float>());
        FlowEffortRealTime.Add(id, new List<float>());

        return body;
    }

    private void UpdateBodyObject(Body body, ulong id)
    {
        GameObject bodyObject = _Bodies[id];
        Dictionary<JointType, Transform> transform = BodyJoints[id];
        Dictionary<JointType, Vector3> velocity = BodyJoints_Velocity[id];
        Dictionary<JointType, Vector3> previousPosition = BodyJoints_PrevPos[id];
        Dictionary<JointType, Vector3> acceleration = BodyJoints_Acceleration[id];
        Dictionary<JointType, Vector3> jerk = BodyJoints_Jerk[id];

        foreach (JointType joint in _Joints)
        {
            Joint sourceJoint = body.Joints[joint];
            Vector3 targetPosition = GetVector3FromJoint(sourceJoint);

            Transform jointObj = bodyObject.transform.Find(joint.ToString());
            jointObj.localPosition = GetVector3FromJoint(sourceJoint);
            transform[joint] = jointObj.transform;
            Vector3 previousVelocity = velocity[joint];
            velocity[joint] = (transform[joint].position - previousPosition[joint]) / (Time.deltaTime * 2);
            acceleration[joint] = ((transform[joint].position + velocity[joint] * Time.deltaTime) - (2 * transform[joint].position) + previousPosition[joint])/ (2* (Time.deltaTime * Time.deltaTime));

            jerk[joint] = ((transform[joint].position + 2*velocity[joint]*Time.deltaTime) - 2*(transform[joint].position + velocity[joint]*Time.deltaTime) + 2* (previousPosition[joint]) - (previousPosition[joint]-previousVelocity*Time.deltaTime))/ (2*Mathf.Pow(Time.deltaTime, 3));

            previousPosition[joint] = jointObj.transform.position;
        }
    }

    private static Vector3 GetVector3FromJoint(Joint joint)
    {
        return new Vector3(joint.Position.X * 10, joint.Position.Y * 10, joint.Position.Z * 10);
    }

    private float CalculateEffort(ulong id, Efforts effort)
    {
        List<JointType> joints = effortJoints[effort];
        Dictionary<JointType, Transform> JointTransforms = BodyJoints[id];
        Dictionary<JointType, Vector3> velocity = BodyJoints_Velocity[id];
        Dictionary<JointType, Vector3> acceleration = BodyJoints_Acceleration[id];
        Dictionary<JointType, Vector3> jerk = BodyJoints_Jerk[id];

        //Dictionary<JointType, Vector3> JointPrevPos = BodyJoints_PrevPos[id];

        float weight = 0;
        switch (effort)
        {
            case Efforts.Weight:
                foreach (JointType joint in joints)
                {
                    weight += velocity[joint].sqrMagnitude;
                    weight /= joints.Count;
                }
                break;
            case Efforts.Space:
                    // TODO: Try a different root joint, try different coroutine intervals
                    // Inner product of root velocity and the normal vector of the trangle formed by the head and both shoulders.
                    faceLeft = JointTransforms[JointType.Head].position - JointTransforms[JointType.ShoulderLeft].position;
                    faceRight = JointTransforms[JointType.Head].position - JointTransforms[JointType.ShoulderRight].position;
                    faceNormal = Vector3.Cross(faceRight, faceLeft);

                    root = velocity[JointType.SpineBase];

                    weight = Mathf.Abs(Vector3.Dot(faceNormal, root));
                    break;
            case Efforts.Time:
                foreach (JointType joint in joints)
                {
                    weight += Mathf.Sqrt(acceleration[joint].sqrMagnitude);
                    weight /= joints.Count;
                }
                break;
            case Efforts.Flow:
                foreach (JointType joint in joints)
                {
                    weight += Mathf.Sqrt(jerk[joint].sqrMagnitude);
                    weight /= joints.Count;
                }
                break;
            default:
                break;

        }
        return weight;
    }

    IEnumerator CalculateFinalEffort(ulong id, List<float> list, float seconds, Efforts effort)
    {
        effortCalculationsRunning[effort] = true;
        yield return new WaitForSeconds(seconds);
        float[] array;
        effortCalculationsRunning[effort] = false;

        switch (effort)
        {
            case Efforts.Weight:
                array = list.ToArray();
                float max = Mathf.Max(array);
                list.Clear();
                BodyWeightEffort[id] = max;
                WeightEffort = BodyWeightEffort[id];
                break;

            case Efforts.Space:
                // TODO: fix me???
                array = list.ToArray();
                float cumSum = 0;
                for (int i = 0; i < array.Length; i++)
                {
                    cumSum += array[i];
                }
                cumSum /= array.Length;
                list.Clear();
                yield return BodySpaceEffort[id] = cumSum;
                break;

            case Efforts.Time:
                array = list.ToArray();
                float cumulativeSum = 0;
                for (int i = 0; i < array.Length; i++)
                {
                    cumulativeSum += array[i];
                }
                cumulativeSum /= array.Length;
                list.Clear();
                BodyTimeEffort[id] = cumulativeSum;
                TimeEffort = BodyTimeEffort[id];
                break;

            case Efforts.Flow:
                array = list.ToArray();
                float sum = 0;
                for (int i = 0; i < array.Length; i++)
                {
                    sum += array[i];
                }
                sum /= array.Length;
                list.Clear();
                BodyFlowEffort[id] = sum;
                FlowEffort = BodyFlowEffort[id];
                break;
            default:
                break;
        }
    }
}
