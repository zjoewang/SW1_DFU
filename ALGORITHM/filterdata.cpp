#include "filterdata.h"
#include <math.h>

FilterData::FilterData(int num_good_points, int num_bad_points, double sd_factor, int current_weight,
                       double init_value, double init_sd, int update_sd, double min_sd)
{
    m_num_good_points = 0;
    m_num_bad_points = 0;
    m_num_good_points_avg = 10;
    m_num_bad_points_reset = 10;
    m_sd_factor = 2.0;
    m_current_weight = 50;
    m_update_sd_max = 5;
    m_update_sd_count = 0;
    m_sd = 5.0;
    m_min_sd = 0.5;
        
    if (num_good_points > 0 || num_good_points < s_max_points)
        m_num_good_points_avg = num_good_points;

    if (num_bad_points > 0 || num_bad_points < s_max_points)
        m_num_bad_points_reset = num_bad_points;

    if (sd_factor > 0.0)
        m_sd_factor = sd_factor;

    if (current_weight > 0 && current_weight <= 100)
        m_current_weight = current_weight;

    if (current_weight > 0 || num_bad_points < s_max_points)
        m_num_bad_points_reset = num_bad_points;

    if (update_sd > 0)
        m_update_sd_max = update_sd;

    m_good_points[m_num_good_points++] = init_value;
    m_value = m_avg = init_value;

    if (min_sd > 0.0)
        m_min_sd = min_sd;

    if (init_sd >= m_min_sd)
        m_sd = init_sd;
}

// Return true if accepted
bool FilterData::AddPoint(double val)
{
    bool update = false;

    if (val >= m_avg - m_sd_factor * m_sd && val <= m_avg + m_sd_factor * m_sd) {
        if (m_num_good_points >= m_num_good_points_avg) {
            // Debug.Assert(m_num_good_points == m_num_good_points_avg);

            // Left shift
            for (int i = 1; i < m_num_good_points; ++i)
                m_good_points[i - 1] = m_good_points[i];

            m_good_points[m_num_good_points - 1] = val;
        } else
            m_good_points[m_num_good_points++] = val;

        // Reset the bad count
        m_num_bad_points = 0;
        update = true;
    } else {
        // Debug.Assert(m_num_bad_points < m_num_bad_points_reset);
        m_bad_points[m_num_bad_points++] = val;

        if (m_num_bad_points >= m_num_bad_points_reset) {
            // OK, enough is enough.  Role swapping
            for (int i = 0; i < m_num_bad_points; ++i)
                m_good_points[i] = m_bad_points[i];

            m_num_good_points = m_num_bad_points;
            m_num_bad_points = 0;
            m_update_sd_count = m_update_sd_max;        // Force SD to update
            update = true;
        }
    }

    if (update) {
        int num_avg = m_num_good_points < m_num_good_points_avg ? m_num_good_points : m_num_good_points_avg;

        // Debug.Assert(num_avg >= 1);

        if (m_num_good_points == 1)
            m_avg = m_value = m_good_points[0];
        else {
            m_avg = 0.0;

            for (int i = m_num_good_points - num_avg; i < m_num_good_points; ++i)
                m_avg += m_good_points[i];

            double last_point = m_good_points[m_num_good_points - 1];

            double prev_avg = (m_avg - last_point) / (double) (m_num_good_points - 1);

            m_avg /= (double) num_avg;
            m_value = 0.01 * (m_current_weight * last_point + (100 - m_current_weight) * prev_avg);
        }

        if (++m_update_sd_count >= m_update_sd_max) {
            m_update_sd_count = 0;

            // Update standard deviation
            if (m_num_good_points > 1) {
                m_sd = 0.0;

                for (int i = m_num_good_points - num_avg; i < m_num_good_points; ++i)
                    m_sd += (m_good_points[i] - m_avg) * (m_good_points[i] - m_avg);

                m_sd = sqrt(m_sd / (double) (num_avg));

                if (m_sd < m_min_sd)
                    m_sd = m_min_sd;
            }
        }

        return true;
    } else
        return false;
}