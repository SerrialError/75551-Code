#include "logger.hpp"

void print_vector(const std::vector<float>& vector, const char* name) {
	printf("%s = [", name);
    for (size_t i = 0; i < vector.size(); ++i) {
        printf("%.4f", vector[i]);
        if (i + 1 < vector.size()) printf(",");
    }
    printf("]\n");	
}
