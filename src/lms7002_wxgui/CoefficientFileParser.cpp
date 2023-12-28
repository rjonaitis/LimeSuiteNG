/**
@file	CoefficientFileParser.cpp
@author	Lime Microsystems
@brief	Coefficient file parser functions
*/

#include "CoefficientFileParser.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <vector>

// ***************************************************************
// Check if 'c' is blank character.
// ***************************************************************
bool Parser::IsBlank(char c)
{
    char blankchar[] = ", \t\n\r";
    for (unsigned i = 0; i < strlen(blankchar); i++)
        if (c == blankchar[i])
            return (true);

    return (false);
}

// ***************************************************************
// Check if 'c' is a digit.
// ***************************************************************
bool Parser::IsDigit(char c)
{
    char digit[] = ".0123456789+-";
    for (unsigned i = 0; i < strlen(digit); i++)
        if (c == digit[i])
            return (true);
    return (false);
}

// ***************************************************************
//	Get integer value from the file
//	Returns:
//		0 upon success,
//		-1 if EOF or
//		-2 if syntax error
// ***************************************************************
int Parser::getValue(FILE* fp, float* v)
{
    char c, c1, str[256];
    int i, opencomments;

    /* Skip blanks, tabs and new lines */
    while (IsBlank(c = fgetc(fp)))
        ;

    /* Its end of file, nothing to read */
    if (c == EOF)
        return (EOF);

    if (c == '/')
    { /* check for comments */
        c1 = fgetc(fp);
        if (c1 == '/')
        { /* C++ one line comment */
            while ((c = fgetc(fp)) != '\n' && c != EOF)
                ;
            if (c == EOF)
                return (EOF);
            else
                return (getValue(fp, v));
        }
        else if (c1 == '*')
        { /* C like comment */
            opencomments = 1;
            c = fgetc(fp);
            if (c == EOF)
                return (EOF);
            while ((c1 = fgetc(fp)) != EOF)
            {
                if (c == '/' && c1 == '*')
                    opencomments++;
                if (c == '*' && c1 == '/')
                    opencomments--;
                c = c1;
                if (opencomments == 0)
                    break;
            }
            if (c1 == EOF)
                return (EOF);
            else
                return (getValue(fp, v));
        }
        else
        {
            ungetc(c1, fp);
        }
    }

    if (IsDigit(c))
    {
        i = 0;
        str[i] = c;
        i++;
        while (IsDigit(c = fgetc(fp)))
        {
            str[i] = c;
            i++;
        }
        ungetc(c, fp);
        str[i] = '\0';
        *v = atof(str);
        return (0);
    }
    else
        return (-2);
}

// ***************************************************************
// Get the coefficients from a file. Return values:
//	-2	syntax error within the file
//	-3	filename is empty string
//	-4	can not open the file
//	-5	too many coefficients in the file
//	>=0 	number of the coefficients read
// ***************************************************************
int Parser::getcoeffs(const char* filename, float* v, int max)
{
    int i, n;
    FILE* fp;

    if (strlen(filename) == 0)
        return (-3);
    if ((fp = fopen(filename, "r")) == NULL)
        return (-4);

    for (n = 0; n < max;)
    {
        i = getValue(fp, v);
        if (i == EOF)
        {
            fclose(fp);
            return (n);
        }
        else if (i == -2)
        {
            fclose(fp);
            return (-2);
        }
        else if (i == 0)
        {
            n++;
            v++;
        }
    }
    fclose(fp);
    return (-5);
}

// ***************************************************************
// Get pair of the coefficients from a file. Return values:
//	-1	odd number of the coefficients found in the file
//	-2	syntax error within the file
//	-3	filename is empty string
//	-4	can not open the file
//	-5	too many coefficients in the file
//	>=0 	number of the coefficients read
// ***************************************************************
int Parser::getcoeffs2(const char* filename, float* v1, float* v2, int max)
{
    int i, n;
    FILE* fp;

    if (strlen(filename) == 0)
        return (-3);
    if ((fp = fopen(filename, "r")) == NULL)
        return (-4);

    for (n = 0; n < max + 1;)
    {
        i = getValue(fp, v1);
        if (i == EOF)
        {
            fclose(fp);
            return (n);
        }
        else if (i == -2)
        {
            fclose(fp);
            return (-2);
        }
        else if (i == 0)
        {
            v1++;
        }

        i = getValue(fp, v2);
        if (i == EOF)
        {
            fclose(fp);
            return (-2);
        }
        else if (i == -2)
        {
            fclose(fp);
            return (-2);
        }
        else if (i == 0)
        {
            v2++;
            n++;
        }
    }

    fclose(fp);
    return (-5);
}

// ***************************************************************
// Saves given coefficients to fir file
// ***************************************************************
void Parser::saveToFile(const std::string& filename, const std::vector<double>& coefficients)
{
    std::ofstream fout;
    fout.open(filename, std::ios::out);

    std::string fname;
    std::size_t name_pos = filename.rfind('\\');

    if (name_pos == std::string::npos)
    {
        name_pos = filename.rfind('/');
    }

    fout << "/* ******************************************************************" << std::endl;
    fout << "   FILE:\t";
    if (name_pos != std::string::npos)
    {
        fname = filename.substr(name_pos + 1);
        fout << fname << std::endl;
    }
    else
    {
        fout << filename << std::endl;
    }

    fout << "   DESCRIPTION:\t" << std::endl;
    fout << "   DATE:\t" << std::endl;
    fout << "   REVISIONS:\t" << std::endl;
    fout << "   ****************************************************************** */" << std::endl << std::endl;

    const std::size_t coefficientCount = coefficients.size();
    for (std::size_t i = 0; i < coefficientCount; ++i)
    {
        fout << "\t" << std::fixed << coefficients[i];

        if (i < coefficientCount - 1) // If not last
        {
            fout << ',' << std::endl;
        }
    }

    fout.close();
}
