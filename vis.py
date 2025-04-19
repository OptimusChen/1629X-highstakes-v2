import statistics
from scipy.stats import norm
from scipy.integrate import quad

# Original data
data = [4.546, 4.5, 4.567, 4.508, 4.602, 4.504, 4.666, 4.662, 4.624, 4.518,
        4.743, 4.748, 4.775, 4.627, 4.642, 4.747, 4.566, 4.551, 4.536, 4.622,
        4.676, 4.557, 4.733, 4.532, 4.667, 4.732, 4.634, 4.7, 4.764, 4.602,
        4.644, 4.77, 4.624, 4.717, 4.538, 4.502, 4.798, 4.772, 4.537, 4.586,
        4.543, 4.563, 4.675, 4.713, 4.509, 4.678, 4.729, 4.782, 4.722, 4.579]

# Apply bias correction
bias = 0.36
corrected_data = [x + bias for x in data]

# New mean and standard deviation
mu = statistics.mean(corrected_data)
sigma = statistics.stdev(corrected_data)

# Define normal PDF with corrected parameters
def corrected_pdf(x):
    return norm.pdf(x, loc=mu, scale=sigma)

print(mu)

# Compute P(4.9 ≤ X ≤ 5.1)
probability, _ = quad(corrected_pdf, 4.85, mu)

# Print result
print("P(4.9 ≤ X ≤ 5.1) after bias correction:", round(probability, 2))
