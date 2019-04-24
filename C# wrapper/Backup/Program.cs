using System;


namespace ExampleTestAppWithCSWrapper
{
	class Program
	{
		static void Main(string[] args)
		{
			CSUnmanagedTestClass testClass = new CSUnmanagedTestClass();
			testClass.PassInt(42);
			testClass.PassString("forty-two");
			string strReturn = testClass.ReturnString();
			System.Console.WriteLine("The sting returned was: " + strReturn);

			testClass.Dispose();
		}
	}
}
