#include "wt.h"
#include "wt_cv.h"

typedef struct
{
    char name[1024];
    q_xyz_quat_type pose;
} chess_results_vrpn_t;

static int load_vrpn_datas(const char *png, chess_results_vrpn_t* dst);

#define CURR_VRPN_LINES 4

typedef struct
{
    char filename[1024];
    q_xyz_quat_type pose;
    chess_results_vrpn_t vrpns[CURR_VRPN_LINES];
} chess_results_t;

std::vector<chess_results_t*> chess_results;

static void chess_results_load(char* yaml)
{
    int a;

    double depth;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<std::string> imageNames;

    // read parse result
    cv::FileStorage fs(yaml, cv::FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs["rvecs"] >> rvecs;
    fs["tvecs"] >> tvecs;
    fs["imageNames"] >> imageNames;
    fs["depth"] >> depth;
    fs.release();

    // translate and load other datas
    for (a = 0; a < rvecs.size(); a++)
    {
        char *tmp, buf[4096];
        q_xyz_quat_type view, pose;
        chess_results_t* r = (chess_results_t*)malloc(sizeof(chess_results_t));

        wt_rvec_tvec_to_quat_xyz(rvecs[a], tvecs[a], &view);

        q_xyz_quat_invert(&pose, &view);

        // to UE axis
        wt_cv_to_ue(&pose, &r->pose);

        // to scene - 7 meters shift
        r->pose.xyz[0] += depth;

        // path where yaml folder
        strncpy(buf, yaml, sizeof(buf));
        tmp = strrchr(buf, '\\');
        if(!tmp)
            tmp = strrchr(buf, '/');
        if (tmp)
            *tmp = 0;
        strcat(buf, "\\");
        strcat(buf, imageNames[a].c_str());

        // save name
        strncpy(r->filename, imageNames[a].c_str(), 1024);

        // build image path and load
        if (load_vrpn_datas(buf, r->vrpns) <= 0)
        {
            fprintf(stderr, "Error, failed to load VRPN datas for [%s]\n", buf);
            exit(1);
        };

        chess_results.push_back(r);
    };
};

static int parse_vrpn_datas(char* buf, chess_results_vrpn_t* dst)
{
    q_type quat;
    q_vec_type yawPitchRoll;
    q_vec_type pos;
    int r;

    r = sscanf(buf, "%s\t%lf,%lf,%lf\t%lf,%lf,%lf\t%lf,%lf,%lf,%lf",
        dst->name,
        &pos[0], &pos[1], &pos[2],
        &yawPitchRoll[0], &yawPitchRoll[1], &yawPitchRoll[2],
        &quat[0], &quat[1], &quat[2], &quat[3]);

    q_normalize(quat, quat);

    if (!strstr(dst->name, "GenericTracker"))
    {
        q_copy(dst->pose.quat, quat);
        q_vec_copy(dst->pose.xyz, pos);
    }
    else
    {
        dst->pose.quat[0] = -quat[2];
        dst->pose.quat[1] = quat[0];
        dst->pose.quat[2] = quat[1];
        dst->pose.quat[3] = quat[3];

        dst->pose.xyz[0] = -pos[2];
        dst->pose.xyz[1] = pos[0];
        dst->pose.xyz[2] = pos[1];
    };

    return r;
};

static int load_vrpn_datas(const char *png, chess_results_vrpn_t* dst)
{
    int r = 0;
    FILE* f;
    char *filename, *ext;

    filename = (char*)malloc(strlen(png) + 2);
    if (!filename)
        return -ENOMEM;

    strcpy(filename, png);
    ext = strrchr(filename, '.');
    if (!ext)
        r = -EFAULT;
    else
    {
        strcpy(ext, ".vrpn");
        f = fopen(filename, "rt");
        if (!f)
            r = -errno;
        else
        {
            while (!feof(f))
            {
                char buf[4096];

                memset(buf, 0, sizeof(buf));
                fgets(buf, sizeof(buf) - 1, f);
                if (!buf[0])
                    continue;

                if (parse_vrpn_datas(buf, dst + r))
                    r++;
            }
        }
    }

    free(filename);
    return r;
}
