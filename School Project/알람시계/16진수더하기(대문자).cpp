#include <iostream>

using namespace std;
int add(int a, int b);
int main(void)
{
	char a = 'A' -0x37;
	char b = 'F'-0x37;
	printf("%d %c\n", a, a);
	printf("%d %c\n", b, b);
	printf("%d %c\n", a-0x30, a - 0x30);
	printf("%d %c\n", b - 0x30, b - 0x30);
	int some = add(a, b);
	printf("%c %d", some, some);
	printf("%c", 10 + 0x37);
	
}
int add(int a, int b)
{
	int sum, carrr;
	if (b == 0)
		return a;
	else if (a == 0)
		return b;
	sum = a ^ b;
	carrr = (a & b) << 1;

	return add(sum, carrr);
}
