#ifndef SIMPLEX_HPP
#define SIMPLEX_HPP

#include "api.h"

// A compact simplex solver for "maximize c^T x subject to Ax <= b, x >= 0"
class Simplex {
public:
    // Solve LP: maximize c^T x, subject to A x <= b, x >= 0
    // A: m x n matrix
    // b: length m
    // c: length n
    // returns optimal x (length n)
    static std::vector<double> solve(const std::vector<std::vector<double>>& A,
                                     const std::vector<double>& b,
                                     const std::vector<double>& c) {
        int m = A.size();
        int n = c.size();
        if (m == 0 || n == 0) throw std::invalid_argument("Empty LP input");

        // tableau has m+1 rows and n+m+1 cols
        std::vector<std::vector<double>> tab(m + 1, std::vector<double>(n + m + 1, 0.0));

        // Fill constraints
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) tab[i][j] = A[i][j];
            tab[i][n + i] = 1.0; // slack
            tab[i][n + m] = b[i];
        }

        // Fill objective (last row)
        for (int j = 0; j < n; j++) tab[m][j] = -c[j];

        // Simplex iterations
        while (true) {
            // Find entering column (most negative in bottom row)
            int col = -1;
            for (int j = 0; j < n + m; j++) {
                if (tab[m][j] < -1e-9 && (col == -1 || tab[m][j] < tab[m][col]))
                    col = j;
            }
            if (col == -1) break; // optimal

            // Find leaving row
            int row = -1;
            for (int i = 0; i < m; i++) {
                if (tab[i][col] > 1e-9) {
                    double ratio = tab[i][n + m] / tab[i][col];
                    if (ratio >= 0 && (row == -1 || ratio < tab[row][n + m] / tab[row][col]))
                        row = i;
                }
            }
            if (row == -1) throw std::runtime_error("Unbounded LP");

            // Pivot
            double piv = tab[row][col];
            for (double &val : tab[row]) val /= piv;
            for (int i = 0; i <= m; i++) {
                if (i == row) continue;
                double factor = tab[i][col];
                for (int j = 0; j <= n + m; j++)
                    tab[i][j] -= factor * tab[row][j];
            }
        }

        // Extract solution
        std::vector<double> x(n, 0.0);
        for (int j = 0; j < n; j++) {
            int pivotRow = -1;
            for (int i = 0; i < m; i++) {
                if (std::abs(tab[i][j] - 1.0) < 1e-9) {
                    bool isPivotCol = true;
                    for (int k = 0; k < m; k++) {
                        if (k != i && std::abs(tab[k][j]) > 1e-9) {
                            isPivotCol = false;
                            break;
                        }
                    }
                    if (isPivotCol) {
                        pivotRow = i;
                        break;
                    }
                }
            }
            if (pivotRow != -1) x[j] = tab[pivotRow][n + m];
        }
        return x;
    }
};

#endif // SIMPLEX_HPP
