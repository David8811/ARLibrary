using System;
using System.Runtime.InteropServices;


namespace GmixonARWrapper
{
	public class GARWrapper : IDisposable
	{
		#region PInvokes


        [DllImport("GmixonARapi.dll")]
        static private extern IntPtr CreateTestClass(char [] object_image_path, int w, int h, int algortihm_type,bool external);

        [DllImport("GmixonARapi.dll")]
		static private extern void DisposeTestClass(IntPtr pTestClassObject);

        [DllImport("GmixonARapi.dll", CharSet = CharSet.Ansi)]
        static private extern void  CallGetTx(IntPtr pTestClassObject, float [] t_vec);

        [DllImport("GmixonARapi.dll", CharSet = CharSet.Ansi)]
        static private extern void CallGetRotMat(IntPtr pTestClassObject, float[] r_mat);



        [DllImport("GmixonARapi.dll", CharSet = CharSet.Ansi)]
        static private extern void CallGetFrame(IntPtr pTestClassObject, char[] r, char[] g, char[] b);


        [DllImport("GmixonARapi.dll")]
		static private extern void CallPassInt(IntPtr pTestClassObject, int nValue);

        [DllImport("GmixonARapi.dll")]
        static private extern bool CallObjectFound(IntPtr pTestClassObject);

        [DllImport("GmixonARapi.dll")]
        static private extern void DoMarkerTracking(IntPtr pTestClassObject,float [] pixels, int lenght);

        [DllImport("GmixonARapi.dll")]
        static private extern void CallSetKalmanFiltering(IntPtr pTestClassObject, bool value);

        [DllImport("GmixonARapi.dll")]
        static private extern void CallSetContourDrawing(IntPtr pTestClassObject, bool value);

        [DllImport("GmixonARapi.dll")]
        static private extern void CallMatFromFloatArray(IntPtr pTestClassObject, float[] pixels, int lenght);

        
		#endregion PInvokes

		#region Members
		private IntPtr m_pNativeObject;		// Variable to hold the C++ class's this pointer
		#endregion Members
                
        public GARWrapper(string object_image_path, int w, int h, int algortihm_type, bool external = false)
		    {
                char[] _path;

                _path = object_image_path.ToCharArray();

			    // We have to Create an instance of this class through an exported function
			    this.m_pNativeObject = CreateTestClass(_path,w,h,algortihm_type,external);
		    }
        
		public void Dispose()
		{
			Dispose(true);
		}

		protected virtual void Dispose(bool bDisposing)
		{
			if(this.m_pNativeObject != IntPtr.Zero)
			{
				// Call the DLL Export to dispose this class
				DisposeTestClass(this.m_pNativeObject);
				this.m_pNativeObject = IntPtr.Zero;
			}

			if(bDisposing)
			{
				// No need to call the finalizer since we've now cleaned
				// up the unmanaged memory
				GC.SuppressFinalize(this);
			}
		}

		// This finalizer is called when Garbage collection occurs, but only if
		// the IDisposable.Dispose method wasn't already called.
        ~GARWrapper()
		{
			Dispose(false);
		}

		#region Wrapper methods
		public void PassInt(int nValue)
		{
			CallPassInt(this.m_pNativeObject, nValue);
		}

		public void DoMarkerTracking(float [] pixels, int lenght)
		{
			DoMarkerTracking(this.m_pNativeObject,pixels,lenght);
		}

        public void GetTx(float [] Tvec)
        {
            
          CallGetTx(this.m_pNativeObject,Tvec);
                                                                                                     
        }

        public void GetRotationMatrix(float[] RMat)
        {

            CallGetRotMat(this.m_pNativeObject, RMat);

        }


        public bool ObjectFound()
        {

           return CallObjectFound(this.m_pNativeObject);

        }

        public void SetKalmanFiltering(bool value)
        {

            CallSetKalmanFiltering(this.m_pNativeObject, value);

        }

        public void SetShowContours(bool value)
        {

            CallSetContourDrawing(this.m_pNativeObject, value);

        }


        public void MatFromArray(float[] pixels, int lenght)
        {

            CallMatFromFloatArray(this.m_pNativeObject, pixels, lenght);

        }

        public void GetFrame(char[] r, char[] g, char[] b)
        {

            CallGetFrame(m_pNativeObject, r, g, b);

        }

		#endregion Wrapper methods
	}
}
