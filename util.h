#ifndef _UTIL_H_
#define _UTIL_H_

#include <string>
#include <vector>
#include <sstream>

using namespace std;

// String tokenizer. From http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html
void Tokenize(const string& str, vector<string>& tokens, const string& delimiters = " ");

//Convert a integer to string
string intToStr(int integer);

//Calculates mean and variance iteratively. New mean and variance are return by reference in mean and var. 
// From https://math.stackexchange.com/questions/374881/recursive-formula-for-variance
void IterativeMeanVariance(double x, unsigned long n, double* mean, double* var);

#endif
