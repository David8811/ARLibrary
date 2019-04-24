using System;
using System.Runtime.InteropServices;


namespace ExampleTestAppWithCSWrapper
{
	public class CSUnmanagedTestClass : IDisposable
	{
		#region PInvokes
		[DllImport("ExampleUnmanagedDLL.dll")]
		static private extern IntPtr CreateTestClass();

		[DllImport("ExampleUnmanagedDLL.dll")]
		static private extern void DisposeTestClass(IntPtr pTestClassObject);

		/*
		[DllImport("ExampleUnmanagedDLL.dll",
			EntryPoint="?PassInt@CUnmanagedTestClass@@QAEXH@Z",
			CallingConvention=CallingConvention.ThisCall)]
		static private extern void PassInt(IntPtr pClassObject, int nValue);
		*/
		[DllImport("ExampleUnmanagedDLL.dll")]
		static private extern void CallPassInt(IntPtr pTestClassObject, int nValue);

		[DllImport("ExampleUnmanagedDLL.dll", CharSet = CharSet.Ansi)]
		static private extern void CallPassString(IntPtr pTestClassObject, string strValue);

		[DllImport("ExampleUnmanagedDLL.dll", CharSet = CharSet.Ansi)]
		static private extern string CallReturnString(IntPtr pTestClassObject);
		#endregion PInvokes

		#region Members
		private IntPtr m_pNativeObject;		// Variable to hold the C++ class's this pointer
		#endregion Members

		public CSUnmanagedTestClass()
		{
			// We have to Create an instance of this class through an exported function
			this.m_pNativeObject = CreateTestClass();
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
		~CSUnmanagedTestClass()
		{
			Dispose(false);
		}

		#region Wrapper methods
		public void PassInt(int nValue)
		{
			CallPassInt(this.m_pNativeObject, nValue);
		}

		public void PassString(string strValue)
		{
			CallPassString(this.m_pNativeObject, strValue);
		}

		public string ReturnString()
		{
			return CallReturnString(this.m_pNativeObject);
		}
		#endregion Wrapper methods
	}
}
