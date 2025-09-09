#include "gap_finding.hpp"

namespace gp
{
    void find_max_gap(const std::vector<float>& ranges, int& start_i, int& end_i)
    {
        int max_length = 0;
        int current_start = -1;

        start_i = -1;
        end_i = -1;

        for (int i = 0; i < ranges.size(); ++i)
        {
            if (ranges[i] > 0.1)  // solo consideramos puntos válidos
            {
                if (current_start == -1)
                    current_start = i;

                // Si es el último punto y seguimos en un gap
                if (i == ranges.size() - 1 && current_start != -1)
                {
                    int length = i - current_start;
                    if (length > max_length)
                    {
                        max_length = length;
                        start_i = current_start;
                        end_i = i;
                    }
                }
            }
            else
            {
                if (current_start != -1)
                {
                    int length = i - current_start;
                    if (length > max_length)
                    {
                        max_length = length;
                        start_i = current_start;
                        end_i = i - 1;
                    }
                    current_start = -1; // reiniciar el inicio del gap
                }
            }
        }
        // Fallback si no se encontró ningún gap válido
        if (start_i == -1 || end_i == -1)
        {
            int mid = ranges.size() / 2;
            start_i = mid;
            end_i = mid;
        }
    }

    void find_best_point(const std::vector<float>& ranges, int& start_i, int& end_i, int& best_index, int max_jump)
        {
            float max_range = 0.0;
            std::vector<int> best_indices;
    
            for (int i = start_i; i <= end_i; i++)
            {
                if (ranges[i] > max_range)
                {
                    max_range = ranges[i];
                    best_indices.clear();
                    best_indices.push_back(i);
                }
                else if (ranges[i] == max_range)
                {
                    best_indices.push_back(i);
                }
            }
    
            // Si no se encontró ningún índice válido, usar el índice medio
            static int previous_best_index = -1;
    
            if (!best_indices.empty())
            {
                int raw_index = best_indices[best_indices.size() / 2];
    
                // Filtro de suavizado
                if (previous_best_index == -1)
                    previous_best_index = raw_index;
        
                // Aplica el filtro suave
                float alpha = 0.1;
                int smoothed_index = static_cast<int>(alpha * raw_index + (1.0 - alpha) * previous_best_index);
        
                // Aplica el límite de salto
                int delta = smoothed_index - previous_best_index;
                if (std::abs(delta) > max_jump)
                {
                    if (delta > 0)
                        smoothed_index = previous_best_index + max_jump;
                    else
                        smoothed_index = previous_best_index - max_jump;
                }
    
                best_index = smoothed_index;
                previous_best_index = smoothed_index;
                }
    
            previous_best_index = best_index;
        }

        //Alternativa 
        int get_best_point(const std::vector<float>& ranges, int start_i, int end_i)
        {
            return (start_i + end_i) / 2;
        }
}
