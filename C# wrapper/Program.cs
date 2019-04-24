using System;
using GmixonARWrapper;


namespace ExampleTestAppWithCSWrapper
{
	class Program
	{
		static void Main(string[] args)
		{
            Console.WriteLine("AR wrapper demo...");
                                   
            GARWrapper myAR = new GARWrapper("F:/objectx.jpg",640,480,6,false);

            myAR.SetShowContours(true);
            myAR.SetKalmanFiltering(false);

            float[] test = new float[640 * 480];

            Random n_generator = new Random();


           
           // myAR.MatFromArray(test, 640*480);
            myAR.DoMarkerTracking(test,test.Length);

                while (true)
                {
                    for (int i = 0; i < 640 * 480; i++)
                        test[i] = (float)n_generator.NextDouble();

                    
                   // myAR.GetFrame(r, g, b);

                    myAR.DoMarkerTracking(null,0);

                    float[] Tvec = new float[3];
                    float[] Rmat = new float[9];

                    myAR.GetTx(Tvec);

                    

                    try
                    {
                        if (myAR.ObjectFound())
                        {
                            Console.WriteLine(Tvec[0] + " " + Tvec[1] + " " + Tvec[2] + " ");

                                                        
                            Console.WriteLine("----------------------------------------------");
                        }

                    }
                    catch (Exception E) { }

                }
					           
            myAR.Dispose();
		}
	}
}
