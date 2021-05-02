import math
import numpy
import pprint
import sys

def make_table(data_set, data, encodings):
    heading_name = []
    for attributes in encodings[0].split(","):
        heading_name.append(attributes)
    data_set[0] = heading_name
    make_list(data_set, data, encodings, heading_name)

def make_list(data_set, data, encodings, heading_name):
    data_list = []
    for row in range(len(data)):
        temp_list = []
        heading_index = 1
        for items in data[row]:
            temp_list.append(encodings[heading_index].split(",")[items])
            heading_index += 1
        data_list.append(temp_list)
    index = 1
    for items in data_list:
        data_set[index] = items
        index += 1

def calculate_entropy(data_set):
    values, count = numpy.unique(data_set[1:, -1], return_counts = True)
    if len(values) == 1:
        return 0
    elif count[0] == count[1]:
        return 1
    else:
        entropy = 0
        total = sum(count)
        for value in count:
            entropy -= (value/total) * math.log(value/total, 2)
        return entropy

def get_index(data_set, attribute):
    shape = data_set.shape
    for column in range(shape[1]):
        if data_set[0][column] == attribute:
            return column

def get_sub_table(data_set, attribute, sub_attribute, encodings):
    shape = data_set.shape
    values, count = numpy.unique(data_set[1:, get_index(data_set, attribute)], return_counts = True)
    row = 0
    column = shape[1]
    for index in range(len(values)):
        if values[index] == sub_attribute:
            row = count[index]
            break
    data_set_temp = numpy.empty([row + 1, column], dtype=numpy.dtype("U100"))
    heading_name = []
    for attributes in encodings[0].split(","):
        heading_name.append(attributes)
    data_set_temp[0] = heading_name
    index = 1
    column = get_index(data_set, attribute)
    for row in range(len(data_set)):
        if data_set[row][column] == sub_attribute:
            data_set_temp[index] = data_set[row]
            index += 1
    return data_set_temp

def calculate_gain(data_set, encodings):
    overall_entropy = calculate_entropy(data_set)
    total_length = len(data_set)
    gain = {}
    attributes = data_set[0][:-1]
    for attribute in attributes:
        sub_attributes = numpy.unique(data_set[1:, get_index(data_set, attribute)])
        gain[attribute] = overall_entropy
        for sub_attribute in sub_attributes:
            sub_entropy = calculate_entropy(get_sub_table(data_set, attribute, sub_attribute, encodings))
            sub_length = len(get_sub_table(data_set, attribute, sub_attribute, encodings))
            gain[attribute] -= (sub_length/total_length) * sub_entropy
    return gain

def max_gain_node(data_set, encodings):
    information_gain = calculate_gain(data_set, encodings)
    return max(information_gain, key=information_gain.get)

def generate_decision(data_set, encodings):
    decision_tree = {}
    most_gain = max_gain_node(data_set, encodings)
    most_gain_attributes = numpy.unique(data_set[1:, get_index(data_set, most_gain)])
    decision_tree[most_gain] = {}
    for attribute in most_gain_attributes:
        sub_table = get_sub_table(data_set, most_gain, attribute, encodings)
        values = numpy.unique(sub_table[1:, -1])
        if len(values) == 1:
            decision_tree[most_gain][attribute] = values[0]
        else:
            decision_tree[most_gain][attribute] = generate_decision(sub_table, encodings)
    return decision_tree

def main():
    data = numpy.loadtxt(sys.argv[1], dtype=int, delimiter=",")
    data_shape = data.shape
    encodings = numpy.loadtxt(sys.argv[2], dtype=str)
    data_set = numpy.empty([data_shape[0] + 1, data_shape[1]], dtype=numpy.dtype("U100"))
    make_table(data_set, data, encodings)
    decision_tree = generate_decision(data_set, encodings)
    pprint.pprint(decision_tree)

main()