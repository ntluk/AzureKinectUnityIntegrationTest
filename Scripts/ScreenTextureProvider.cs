using Microsoft.Azure.Kinect.Sensor;
using System.Threading.Tasks;
using UnityEngine;

public class ScreenTextureProvider : MonoBehaviour {

    [SerializeField]
    public Material depthMat;
    public Material colorMat;

    [SerializeField]
    public float foregroundBoundry;

    [SerializeField]
    public float backgroundBoundry;

    private Texture2D depthTexture;
    private Texture2D colorTexture;

    //Variable for handling Kinect
    Device kinect;
    //Number of all points of PointCloud 
    int num;
    //Used to draw a set of points
    Mesh mesh;
    //Array of coordinates for each point in PointCloud
    Vector3[] vertices;
    //Array of colors corresponding to each point in PointCloud
    Color32[] colors;
    Color32[] depthValues;
    //List of indexes of points to be rendered
    int[] indices;
    //Class for coordinate transformation(e.g.Color-to-depth, depth-to-xyz, etc.)
    Transformation transformation;

    void Start() {
        //The method to initialize Kinect
        InitKinect();
        InitMesh();

        depthValues = new Color32[921600];  // = num
        depthTexture = new Texture2D(1280, 720);
        colorTexture = new Texture2D(1280, 720);

        //Loop to get data from Kinect and rendering
        Task t = KinectLoop();
    }

    //Initialization of Kinect
    private void InitKinect() {
        //Connect with the 0th Kinect
        kinect = Device.Open(0);
        //Setting the Kinect operation mode and starting it
        kinect.StartCameras(new DeviceConfiguration {
            ColorFormat = ImageFormat.ColorBGRA32,
            ColorResolution = ColorResolution.R720p,
            DepthMode = DepthMode.NFOV_Unbinned,
            SynchronizedImagesOnly = true,
            CameraFPS = FPS.FPS30
        });
        //Access to coordinate transformation information
        transformation = kinect.GetCalibration().CreateTransformation();
    }
    //Prepare to draw point cloud.
    private void InitMesh() {
        //Get the width and height of the Depth image and calculate the number of all points
        int width = kinect.GetCalibration().ColorCameraCalibration.ResolutionWidth;
        int height = kinect.GetCalibration().ColorCameraCalibration.ResolutionHeight;
        num = width * height;

        //Debug.Log(num);
        ////Instantiate mesh
        //mesh = new Mesh();
        //mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        //Allocation of vertex and color storage space for the total number of pixels in the depth image
        vertices = new Vector3[num];
        colors = new Color32[num];
        indices = new int[num];

        //Initialization of index list
        for (int i = 0; i < num; i++) {
            indices[i] = i;
        }

        ////Allocate a list of point coordinates, colors, and points to be drawn to mesh
        //mesh.vertices = vertices;
        //mesh.colors32 = colors;
        //mesh.SetIndices(indices, MeshTopology.Points, 0);

        //gameObject.GetComponent<MeshFilter>().mesh = mesh;
    }
    private async Task KinectLoop() {
        while (true) {
            using (Capture capture = await Task.Run(() => kinect.GetCapture()).ConfigureAwait(true)) {

                //Getting depth and color information
                Image depthImage = transformation.DepthImageToColorCamera(capture.Depth);
                Image colorImage = capture.Color;

                BGRA[] colorArray = colorImage.GetPixels<BGRA>().ToArray();

                Image xyzImage = transformation.DepthImageToPointCloud(capture.Depth);
                Short3[] xyzArray = xyzImage.GetPixels<Short3>().ToArray();

                for (int i = 0; i < num; i++) {
                    vertices[i].x = xyzArray[i].X * 0.001f;
                    vertices[i].y = -xyzArray[i].Y * 0.001f;//reverse
                    vertices[i].z = xyzArray[i].Z * 0.001f;

                    float norm = xyzArray[i].Z * 0.001f / 10.0f;

                    byte depth = 0;

                    if (norm >= foregroundBoundry && norm <= backgroundBoundry) {

                        depth = (byte)(255 - (norm * 255));
                    }

                    depthValues[i].g = depth;
                    depthValues[i].r = depth;
                    depthValues[i].b = depth;
                    depthValues[i].a = 255;

                    colors[i].b = colorArray[i].B;
                    colors[i].g = colorArray[i].G;
                    colors[i].r = colorArray[i].R;
                    colors[i].a = 255;
                }

                //set pixels for textures ans apply changes
                depthTexture.SetPixels32(depthValues);
                depthTexture.Apply(true, false);

                colorTexture.SetPixels32(colors);
                colorTexture.Apply(true, false);

                // set main textures of materials
                depthMat.mainTexture = depthTexture;
                colorMat.mainTexture = colorTexture;
                
                //mesh.vertices = vertices;
                //mesh.colors32 = colors;
                //mesh.RecalculateBounds();
            }
        }
    }

    //Stop Kinect as soon as this object disappear
    private void OnDestroy() {
        kinect.StopCameras();
    }
}