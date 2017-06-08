 class FilterData
{
    private:
        const static int s_max_points = 100;
        double m_good_points[s_max_points];
        double m_bad_points[s_max_points];
        int m_num_good_points;
        int m_num_bad_points;
        int m_num_good_points_avg;
        int m_num_bad_points_reset;
        double m_sd_factor;
        int m_current_weight;
        int m_update_sd_max;
        int m_update_sd_count;
        double m_avg;
        double m_sd;
        double m_min_sd;
        double m_value;
    
    public:
        FilterData(int num_good_points, int num_bad_points, double sd_factor, int current_weight,
                   double init_value, double init_sd, int update_sd, double min_sd);
        bool AddPoint(double val);
        double GetValue() { return m_value; }
};
    