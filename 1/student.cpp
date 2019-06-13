/* students.cpp */
#include <string>
#include <fstream>
#include "student.h"

// Конструктор Students
Students::Students(std::string name, std::string last_name)
{
	Students::set_name(name);
	Students::set_last_name(last_name);
}

// Установка имени студента
void Students::set_name(std::string student_name)
{
	Students::name = student_name;
}

// Получение имени студента
std::string Students::get_name()
{
	return Students::name;
}

// Установка фамилии студента
void Students::set_last_name(std::string student_last_name)
{
	Students::last_name = student_last_name;
}

// Получение фамилии студента
std::string Students::get_last_name()
{
	return Students::last_name;
}

// Установка промежуточных оценок
void Students::set_scores(int scores[])
{
	int sum = 0;
	for (int i = 0; i < 5; ++i) {
		Students::scores[i] = scores[i];
		sum += scores[i];
	}
}

// Установка среднего балла
void Students::set_average_ball(float ball)
{
	Students::average_ball = ball;
}

// Получение среднего балла
float Students::get_average_ball()
{
	return Students::average_ball;
}