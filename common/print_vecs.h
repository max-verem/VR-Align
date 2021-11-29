static void zero_matrix(q_matrix_type m)
{
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) m[i][j] = 0.0;
}

static void print_vec(const char* name, const q_vec_type vec)
{
    printf("%s = [%f, %f, %f]\n", name, vec[0], vec[1], vec[2]);
}

static void print_quat(const char* name, const q_type vec)
{
    printf("%s = [%f, %f, %f, %f]\n", name, vec[0], vec[1], vec[2], vec[3]);
}

static void print_yawPitchRoll(const char* name, const q_type quat)
{
    q_vec_type yawPitchRoll;

    q_to_euler(yawPitchRoll, quat);
    printf("%s(yawPitchRoll) = [%f', %f', %f']\n", name,
        Q_RAD_TO_DEG(yawPitchRoll[0]),
        Q_RAD_TO_DEG(yawPitchRoll[1]),
        Q_RAD_TO_DEG(yawPitchRoll[2]));
}

static void print_quat2(const char* name, const q_type quat)
{
    q_vec_type yawPitchRoll;

    q_to_euler(yawPitchRoll, quat);

    printf("%s = [%f, %f, %f, %f] {%f', %f', %f' } \n",
        name, quat[0], quat[1], quat[2], quat[3],
        Q_RAD_TO_DEG(yawPitchRoll[0]),
        Q_RAD_TO_DEG(yawPitchRoll[1]),
        Q_RAD_TO_DEG(yawPitchRoll[2]));
}

static void print_xyz_quat(const char* name, const q_xyz_quat_type *trg)
{
    q_vec_type yawPitchRoll;

    q_to_euler(yawPitchRoll, trg->quat);

    printf("%s = [%6.3f, %6.3f, %6.3f] @ {%10.5f', %10.5f', %10.5f'} [%8.5f, %8.5f, %8.5f, %8.5f]\n",
        name,
        trg->xyz[0], trg->xyz[1], trg->xyz[2],
        Q_RAD_TO_DEG(yawPitchRoll[0]),
        Q_RAD_TO_DEG(yawPitchRoll[1]),
        Q_RAD_TO_DEG(yawPitchRoll[2]),
        trg->quat[0], trg->quat[1], trg->quat[2], trg->quat[3]
    );
}

static void print_matrix(const char* name, q_matrix_type m)
{
    printf("%s =\n", name);
    for (int row = 0; row < 4; row++)
    {
        printf("    |");
        for (int col = 0; col < 4; col++) printf("%6.3f ", m[col][row]);
        printf("|\n");
    }
};
