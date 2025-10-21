#ifndef SIMPLEX_HPP
#define SIMPLEX_HPP

#include "api.h"

class Simplex {
public:
    // Solve LP: maximize c^T x, subject to A x <= b, x free (unrestricted) (we allow negative A, b, x)
    // A: m x n matrix
    // b: length m
    // c: length n
    // returns optimal x (length n) corresponding to original variables (not split ones)
    static std::vector<double> solve(const std::vector<std::vector<double>>& A_in,
                                     const std::vector<double>& b_in,
                                     const std::vector<double>& c_in) {
        const double EPS = 1e-9;
        int m = (int)A_in.size();
        int n = (int)c_in.size();
        if (m == 0 || n == 0) throw std::invalid_argument("Empty LP input");

        // ---- 1) Handle unrestricted original variables by splitting:
        // For each original x_j (free), create x_j_pos and x_j_neg, both >= 0, with x_j = pos - neg.
        // New variable count:
        int n2 = 2 * n;

        // Build A' (m x n2) and c' (n2)
        std::vector<std::vector<double>> A(m, std::vector<double>(n2, 0.0));
        std::vector<double> c(n2, 0.0);
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                A[i][2*j]     =  A_in[i][j];    // x_j_pos coef
                A[i][2*j + 1] = -A_in[i][j];    // x_j_neg coef (because x = pos - neg)
            }
        }
        for (int j = 0; j < n; ++j) {
            c[2*j]     =  c_in[j];   // objective c^T (pos)
            c[2*j + 1] = -c_in[j];   // objective for neg part is -c
        }

        // ---- 2) Build constraint rows with slacks/surplus and possibly artificials.
        // We'll convert each constraint (originally A_in x <= b_in) into an equality using:
        // - if b >= 0: keep <= and add slack (makes easy basic var)
        // - if b < 0: multiply row by -1 => it becomes >= with new rhs >= 0
        // For >= constraints we add a surplus (-s) and an artificial variable (a) which will be in basis.
        //
        // We'll track columns: [ original_vars (n2) | slacks | surplus (not kept separately) | artificials ] + RHS

        std::vector<int> rowType(m, 0); // 0: originally <= and b>=0 (slack), 1: >= (we will add artificial), 2: equality (artificial)
        std::vector<std::vector<double>> rows(m);
        std::vector<double> rhs(m);

        for (int i = 0; i < m; ++i) {
            // copy A row
            rows[i] = A[i];
            double b = b_in[i];
            if (b >= 0) {
                // keep row as <=
                rowType[i] = 0; // slack
                rhs[i] = b;
            } else {
                // multiply by -1 => becomes >= with positive rhs
                rowType[i] = 1; // will need surplus + artificial
                for (double &val : rows[i]) val = -val;
                rhs[i] = -b;
            }
        }

        // Count slack and artificial vars
        int slack_count = 0;
        int art_count = 0;
        for (int i = 0; i < m; ++i) {
            if (rowType[i] == 0) slack_count++;
            else art_count++; // for >= we will add an artificial (and a surplus which is not basic)
        }

        int totalCols = n2 + slack_count + art_count; // exclude RHS
        int slack_start = n2;
        int art_start = n2 + slack_count;

        // Build full tableau (m constraint rows + 1 objective row) with totalCols + 1 (RHS)
        int cols = totalCols + 1;
        std::vector<std::vector<double>> tab(m + 1, std::vector<double>(cols, 0.0));
        // basis: for each row, which column is basic
        std::vector<int> basis(m, -1);

        // Fill constraint rows
        int s_idx = 0;
        int a_idx = 0;
        for (int i = 0; i < m; ++i) {
            // copy variable columns
            for (int j = 0; j < n2; ++j) tab[i][j] = rows[i][j];

            if (rowType[i] == 0) {
                // add slack variable with coefficient +1 and make it basic
                tab[i][slack_start + s_idx] = 1.0;
                basis[i] = slack_start + s_idx;
                s_idx++;
            } else {
                // rowType == 1 (>= after flipping): add surplus (coefficient -1) and an artificial (+1)
                // we won't keep surplus as basic; artificial will be basic
                // Surplus column is not tracked separately as a basic var; we simply put -1 in some column if we wanted.
                // For simplicity, we omit creating an explicit surplus column and only add artificial column with +1
                // and put a -1 where the surplus would be into the artificial's handling is standard.
                // We'll add artificial var column and make it basic.
                tab[i][art_start + a_idx] = 1.0; // artificial
                basis[i] = art_start + a_idx;
                a_idx++;
                // Note: the surplus column could be added if you want explicit variable indexing of surplus vars.
            }

            // RHS
            tab[i][cols - 1] = rhs[i];
        }

        // ---- Phase I setup: objective is minimize sum(artificials).
        // We will do Phase I only if art_count > 0 (if art_count==0 then feasible start already).
        auto pivot = [&](int prow, int pcol, int rowsTot, int colsTot, std::vector<std::vector<double>>& T, std::vector<int>& basisRef) {
            double piv = T[prow][pcol];
            if (std::abs(piv) < EPS) throw std::runtime_error("Numerical instability (pivot too small)");
            // normalize pivot row
            for (int j = 0; j < colsTot; ++j) T[prow][j] /= piv;
            // eliminate other rows
            for (int i = 0; i < rowsTot; ++i) {
                if (i == prow) continue;
                double fac = T[i][pcol];
                if (std::abs(fac) < EPS) continue;
                for (int j = 0; j < colsTot; ++j) T[i][j] -= fac * T[prow][j];
            }
            basisRef[prow] = pcol;
        };

        auto simplex_loop = [&](std::vector<std::vector<double>>& T, std::vector<int>& basisRef, int rowsTot, int colsTot) -> void {
            // T has rowsTot rows, colsTot columns (including RHS at colsTot-1). last row is objective row.
            while (true) {
                // find entering column: most negative in objective row (we keep the same convention: T[last][j] < -EPS)
                int last = rowsTot - 1;
                int enter = -1;
                for (int j = 0; j < colsTot - 1; ++j) {
                    if (T[last][j] < -EPS && (enter == -1 || T[last][j] < T[last][enter])) enter = j;
                }
                if (enter == -1) break; // optimal

                // ratio test for leaving row
                int leave = -1;
                double bestRatio = 0.0;
                for (int i = 0; i < rowsTot - 1; ++i) {
                    if (T[i][enter] > EPS) {
                        double ratio = T[i][colsTot - 1] / T[i][enter];
                        if (ratio >= -EPS && (leave == -1 || ratio < bestRatio - EPS || (std::abs(ratio - bestRatio) < EPS && basisRef[i] < basisRef[leave]))) {
                            leave = i;
                            bestRatio = ratio;
                        }
                    }
                }
                if (leave == -1) throw std::runtime_error("Unbounded LP (during simplex loop)");
                pivot(leave, enter, rowsTot, colsTot, T, basisRef);
            }
        };

        // If there are artificial vars, prepare Phase I objective
        if (art_count > 0) {
            // bottom row stores -c (as in your original code). For Phase I, objective we want to maximize - sum(artificials)
            // so c_phase1_j = -1 for artificial columns, hence bottom entries = -c_phase1 = +1 for art columns.
            int last = m;
            for (int j = 0; j < cols; ++j) tab[last][j] = 0.0;

            // set bottom row (initial -c_phase1)
            for (int j = art_start; j < art_start + art_count; ++j) {
                tab[last][j] = 1.0; // -c_phase1 = -(-1) = +1
            }
            tab[last][cols - 1] = 0.0;

            // For every artificial that is currently BASIC (we set them as basic for those rows),
            // we must zero its reduced cost by doing bottom += c_b * row, where c_b = -1 (coefficient in phase1 objective)
            // c_b = -1 => bottom += -1 * row => bottom -= row
            for (int i = 0; i < m; ++i) {
                int bcol = basis[i];
                if (bcol >= art_start && bcol < art_start + art_count) {
                    // c_b = -1
                    double c_b = -1.0;
                    for (int j = 0; j < cols; ++j) tab[last][j] += c_b * tab[i][j];
                }
            }

            // Run Phase I simplex
            simplex_loop(tab, basis, m + 1, cols);

            double phase1_obj = tab[m][cols - 1]; // bottom RHS is the maximized value of (-sum(art))
            // recall: Phase I objective w* = -phase1_obj
            if (phase1_obj < -EPS) {
                // phase1_obj negative means w* > 0 => infeasible
                throw std::runtime_error("LP is infeasible (Phase I found positive artificial sum)");
            }

            // Remove artificial columns from tableau.
            // If an artificial is in basis, try to pivot it out using a non-artificial nonzero column in that row.
            for (int a = 0; a < art_count; ++a) {
                int acol = art_start + a;
                // If artificial column is basic in some row, remove it by pivoting if possible
                int prow = -1;
                for (int i = 0; i < m; ++i) if (basis[i] == acol) prow = i;
                if (prow != -1) {
                    // find a nonzero column in this row that's not artificial to pivot on
                    int newcol = -1;
                    for (int j = 0; j < art_start; ++j) {
                        if (std::abs(tab[prow][j]) > EPS) {
                            newcol = j;
                            break;
                        }
                    }
                    if (newcol != -1) {
                        pivot(prow, newcol, m + 1, cols, tab, basis);
                    } else {
                        // entire row zero in non-art columns; artificial is zero variable and can be removed safely.
                        // mark basis as -1 and continue; the column will be removed below.
                        basis[prow] = -1;
                    }
                }
            }

            // Now physically remove artificial columns by creating a new tableau without them
            int newTotalCols = art_start; // drop everything from art_start .. art_start+art_count-1
            int newCols = newTotalCols + 1;
            std::vector<std::vector<double>> newTab(m + 1, std::vector<double>(newCols, 0.0));
            std::vector<int> newBasis(m, -1);
            for (int i = 0; i < m + 1; ++i) {
                for (int j = 0; j < newTotalCols; ++j) {
                    newTab[i][j] = tab[i][j];
                }
                newTab[i][newCols - 1] = tab[i][cols - 1];
            }
            // translate basis indices (anything >= art_start now removed)
            for (int i = 0; i < m; ++i) {
                if (basis[i] == -1) newBasis[i] = -1;
                else if (basis[i] < art_start) newBasis[i] = basis[i];
                else newBasis[i] = -1; // should not happen after our pivot attempts; but leave -1 if so
            }
            tab.swap(newTab);
            basis.swap(newBasis);
            cols = newCols;
            totalCols = newTotalCols;
            // slack_start and art_start no longer needed
        }

        // ---- Phase II: set objective to original c (on transformed variables).
        // bottom row stores -c (as your original code). Build bottom row and then fix it using basis.
        int last_row = m;
        for (int j = 0; j < cols; ++j) tab[last_row][j] = 0.0;
        // c has length n2; we must copy to columns 0..n2-1 (some columns may have been removed if artificials existed)
        for (int j = 0; j < (int)c.size() && j < cols - 1; ++j) tab[last_row][j] = -c[j];
        tab[last_row][cols - 1] = 0.0;

        // For each basic variable with index bcol and corresponding objective coefficient c_b (we need c vector extended by zeros for slacks),
        // add c_b * row_i to bottom row to make reduced costs correct. c_b for slack columns is 0.
        for (int i = 0; i < m; ++i) {
            int bcol = basis[i];
            double c_b = 0.0;
            if (bcol >= 0 && bcol < (int)c.size()) c_b = c[bcol];
            // else slack columns have c_b = 0
            if (std::abs(c_b) > 0.0) {
                for (int j = 0; j < cols; ++j) tab[last_row][j] += c_b * tab[i][j];
            }
        }

        // Run Phase II simplex
        simplex_loop(tab, basis, m + 1, cols);

        // Extract solution for original variables (combine pos-neg pairs)
        std::vector<double> x_orig(n, 0.0);
        for (int j = 0; j < n; ++j) {
            int col_pos = 2*j;
            int col_neg = 2*j + 1;
            double val_pos = 0.0, val_neg = 0.0;
            int pivotRow = -1;
            // find if col_pos is basic
            for (int i = 0; i < m; ++i) {
                if (basis[i] == col_pos) { pivotRow = i; break; }
            }
            if (pivotRow != -1) val_pos = tab[pivotRow][cols - 1];
            // find if col_neg is basic
            pivotRow = -1;
            for (int i = 0; i < m; ++i) {
                if (basis[i] == col_neg) { pivotRow = i; break; }
            }
            if (pivotRow != -1) val_neg = tab[pivotRow][cols - 1];
            x_orig[j] = val_pos - val_neg;
        }

        return x_orig;
    }
};

#endif // SIMPLEX_HPP
