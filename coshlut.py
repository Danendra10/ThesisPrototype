import math

def generate_cosh_lut_cpp(min_input, max_input, step, filename):
    with open(filename, 'w') as file:
        file.write("const double cosh_lut[] = {")

        current_input = min_input
        while current_input <= max_input:
            cosh_value = math.cosh(current_input)
            file.write(f"{cosh_value}, ")
            current_input += step

        file.seek(file.tell() - 2)  # Remove the last comma and space
        file.write("};\n")

# Generate the LUT
min_input = -26.0
max_input = 26.0
step = 0.01
generate_cosh_lut_cpp(min_input, max_input, step, "cosh_lut.cpp")