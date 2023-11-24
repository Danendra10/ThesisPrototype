import math

def generate_tanh_lut_cpp(min_input, max_input, step, filename):
    with open(filename, 'w') as file:
        file.write("const double tanh_lut[] = {")

        current_input = min_input
        while current_input <= max_input:
            tanh_value = math.tanh(current_input)
            file.write(f"{tanh_value}, ")
            current_input += step

        file.seek(file.tell() - 2)  # Remove the last comma and space
        file.write("};\n")

# Generate the LUT
min_input = -26.0
max_input = 26.0
step = 0.01
generate_tanh_lut_cpp(min_input, max_input, step, "tanh_lut.cpp")