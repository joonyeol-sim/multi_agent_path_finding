with open("atasks.txt") as f:
    # read the file line by line
    lines = f.readlines()
    # remove the newline character
    lines = [line.strip() for line in lines]
    # split the line by space
    lines = [line.split() for line in lines]
    # convert the string to int

lines = lines[1:]
min_num_of_task = 1000000
max_num_of_task = 0
sum_num_of_task = 0
sum_finished_time_list = []

for line in lines:
    for all_task in line:
        task_list = all_task[:-1].split(";")
        finished_task = 0
        sum_finished_time = 0
        prev_finished_time = 0
        for each_task in task_list[1:]:
            location, finish_time, _ = each_task.split(",")
            if finish_time != "-1":
                finished_task += 1
                sum_finished_time += int(finish_time) - prev_finished_time
                prev_finished_time = int(finish_time)
        if finished_task < min_num_of_task:
            min_num_of_task = finished_task
        if finished_task > max_num_of_task:
            max_num_of_task = finished_task
        sum_num_of_task += finished_task
        sum_finished_time_list.append(sum_finished_time / finished_task)

print(min_num_of_task, max_num_of_task, sum_num_of_task / len(lines))
print(
    min(sum_finished_time_list),
    max(sum_finished_time_list),
    sum(sum_finished_time_list) / len(lines),
)
