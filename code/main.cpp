/*
 For more details on Win32 check MSDN.
 The windows initalization code is taken from here https://msdn.microsoft.com/en-us/library/bb384843(v=vs.120).aspx
 Go to that link for a more detailed explanation of workign with the Win32 API
*/
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <tchar.h>
#include <io.h>
#include <stdio.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>

//Include GLEW. Used for handling OpenGL extensions. http://glew.sourceforge.net/
#define GLEW_STATIC
#include <GL\glew.h>

//Include OpenGL for rendering
#include <GL\gl.h>

//I didn't do the math library homework, so I'm using glm as an alternative. 
//It's open source so you can use it as a reference
#include <glm\glm.hpp>

//This is the file that contains most of the funcitonal and platform-independant code
#include "Engine.h"
Engine m_engine;

//Static consts for later
static TCHAR szWindowClass[] = _T("orbitalsApp");
static TCHAR szTitle[] = _T("Orbitals");

void EnableConsole()
{
	//Create a console for this application
	AllocConsole();
	//Redirect unbuffered STDOUT to the console
	HANDLE ConsoleOutput = GetStdHandle(STD_OUTPUT_HANDLE);
	int SystemOutput = _open_osfhandle(intptr_t(ConsoleOutput), _O_TEXT);
	FILE *COutputHandle = _fdopen(SystemOutput, "w");
	*stdout = *COutputHandle;
	setvbuf(stdout, NULL, _IONBF, 0);

	//Redirect unbuffered STDERR to the console
	HANDLE ConsoleError = GetStdHandle(STD_ERROR_HANDLE);
	int SystemError = _open_osfhandle(intptr_t(ConsoleError), _O_TEXT);
	FILE *CErrorHandle = _fdopen(SystemError, "w");
	*stderr = *CErrorHandle;
	setvbuf(stderr, NULL, _IONBF, 0);

	//Redirect unbuffered STDIN to the console
	HANDLE ConsoleInput = GetStdHandle(STD_INPUT_HANDLE);
	int SystemInput = _open_osfhandle(intptr_t(ConsoleInput), _O_TEXT);
	FILE *CInputHandle = _fdopen(SystemInput, "r");
	*stdin = *CInputHandle;
	setvbuf(stdin, NULL, _IONBF, 0);

	//make cout, wcout, cin, wcin, wcerr, cerr, wclog and clog point to console as well
	std::ios::sync_with_stdio(true);
}

//Our window will recieve messages from button clicks and the system.
//This is handled by the window procedure function below
LRESULT CALLBACK WndProc(
	HWND hWnd,        // handle to window
	UINT message,        // message identifier
	WPARAM wParam,    // first message parameter
	LPARAM lParam)    // second message parameter
{
	//TODO: Explain
	PAINTSTRUCT ps;
	HDC hdc;
	TCHAR greeting[] = _T("Hello, World!");

	switch (message)
	{
		case WM_KEYDOWN:
		{
			switch (wParam)
			{
				case 'W':
					m_engine.SetKey(Orbitals::Input::InputKey::Forward, true);
					break;
				case 'S':
					m_engine.SetKey(Orbitals::Input::InputKey::Backwards, true);
					break;
				case 'A':
					m_engine.SetKey(Orbitals::Input::InputKey::Left, true);
					break;
				case 'D':
					m_engine.SetKey(Orbitals::Input::InputKey::Right, true);
					break;
			}
			break;
		}
		case WM_KEYUP:
		{
			switch (wParam)
			{
			case 'W':
				printf("W up \n");
				m_engine.SetKey(Orbitals::Input::InputKey::Forward, false);
				break;
			case 'S':
				m_engine.SetKey(Orbitals::Input::InputKey::Backwards, false);
				break;
			case 'A':
				m_engine.SetKey(Orbitals::Input::InputKey::Left, false);
				break;
			case 'D':
				m_engine.SetKey(Orbitals::Input::InputKey::Right, false);
				break;
			}
			break;
		}
		case WM_MOUSEMOVE:
		{
			auto mousePos = MAKEPOINTS(lParam);
			m_engine.SetMousePosition(mousePos.x, mousePos.y);
			break;
		}
		case WM_LBUTTONDOWN:
		{
			m_engine.SetLeftBtn(true);
			break;
		}
		case WM_LBUTTONUP:
		{
			m_engine.SetLeftBtn(false);
			break;
		}
		case WM_PAINT:
			break;
		case WM_SIZE:
			glViewport(0, 0, LOWORD(lParam), HIWORD(lParam));
			break;
		case WM_DESTROY:
			PostQuitMessage(0);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
			break;
    }

	return 0;
}

//Entry point. Win32 application use WinMain() instead of main()
int WINAPI WinMain(		 //https://msdn.microsoft.com/library/windows/desktop/ms633559
	HINSTANCE hInstance, 	 //Handle to instance of the app
	HINSTANCE hPrevInstance, //Previous app handle, always Null
	LPSTR lpCmdLine,	 //Command line arguements
	int nCmdShow)		 //How to show window
{

	EnableConsole();
	  //The following is the structure we will use to create out window.
	WNDCLASSEX wcex; //https://msdn.microsoft.com/en-us/library/ms633577(v=vs.120).aspx

	wcex.cbSize         = sizeof(WNDCLASSEX); //size of the struct
	wcex.style          = CS_HREDRAW | CS_VREDRAW | CS_OWNDC; //Style of the window. We combine using binary or.
															//https://msdn.microsoft.com/en-us/library/ff729176(v=vs.85).aspx
	wcex.lpfnWndProc    = WndProc; //Function pointer to our window procedure. See above
	wcex.cbClsExtra     = 0;       //Extra bytes to allocate following the struct.
	wcex.cbWndExtra     = 0;       //Extra bytes to allocate follwing the window instance
	wcex.hInstance      = hInstance; //HAndle to the instance
	wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_APPLICATION)); //Handle to our window icon. We use the default one. 
	wcex.hCursor        = LoadCursor(NULL, IDC_ARROW);//Handle to our cursor icon. We use the default one.
	wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1); //Handle to the brush used to  paint the background. It can be a brush or a color value.
	wcex.lpszMenuName   = NULL; //Pointer to the name of the class menu as it appears in the resource menu.
	wcex.lpszClassName  = szWindowClass; //Name used to identify the class.
	wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_APPLICATION)); //Small icon for the window.

	//Next we need to register the class. This is so the Windows API can work with the class.
	//We handle the case of falure by displaying a message box.
	if (!RegisterClassEx(&wcex)) //This function returns an ATOM.
	{
		MessageBox( //Desplay a modal dialog box https://msdn.microsoft.com/en-us/library/windows/desktop/ms645505(v=vs.85).aspx
			NULL, //Handle to owner
			_T("Call to RegisterClassEx failed!"), //Message
			_T("Orbitals"), //Title
			NULL); //Contents: buttons, icons etc.

		return 1;
	}

	//Next we create the window
	HWND hWnd = CreateWindow(               //https://msdn.microsoft.com/en-us/library/ms632679(v=vs.110).aspx
				szWindowClass, //Registerd class name
				szTitle,       //Title of window
				WS_OVERLAPPEDWINDOW, //Style of the window. https://msdn.microsoft.com/en-us/library/ms632600(v=vs.85).aspx
				CW_USEDEFAULT, //x coordinate, we use the default.
				CW_USEDEFAULT, //y coordinate, we use the default.
				1024,           //Width
				640,           //Height
				NULL,          //Parent window.
				NULL,          //Handle to a defined menu
				hInstance,     //Handle to the instance of the app
				NULL           //Message sent to the window before construction is done
				);
	if (!hWnd)
	{
		MessageBox(NULL,
			_T("Call to CreateWindow failed!"),
			_T("Win32 Guided Tour"),
			NULL);
      
		return 1;
	}

	//Initialize OpenGL context.Go to http://nightbug.us/index.php/OpenGL_on_Win32_API_and_GLEW for more details
	HDC hdc; //Handle to the device context
	HGLRC hglrc; //Handle to the GL rendering context

	PIXELFORMATDESCRIPTOR pfd = //Describes the pixel format for drawing. https://msdn.microsoft.com/en-us/library/windows/desktop/dd368826(v=vs.85).aspx
	{
		sizeof(PIXELFORMATDESCRIPTOR), //Size of the pfd
		1,                      //version
		PFD_DRAW_TO_WINDOW |   // support window  
		PFD_SUPPORT_OPENGL |   // support OpenGL  
		PFD_DOUBLEBUFFER,      // double buffered  
		PFD_TYPE_RGBA,         // RGBA type  
		24,                    // 24-bit color depth  
		0, 0, 0, 0, 0, 0,      // color bits ignored  
		0,                     // no alpha buffer  
		0,                     // shift bit ignored  
		0,                     // no accumulation buffer  
		0, 0, 0, 0,            // accum bits ignored  
		32,                    // 32-bit z-buffer  
		0,                     // no stencil buffer  
		0,                     // no auxiliary buffer  
		PFD_MAIN_PLANE,        // main layer  
		0,                     // reserved  
		0, 0, 0                // layer masks ignored  
	};
  
	hdc = GetDC(hWnd); // get the handle to our window's device context
	SetPixelFormat(hdc, ChoosePixelFormat(hdc, &pfd), &pfd);
	hglrc = wglCreateContext(hdc); //Create context
	wglMakeCurrent(hdc, hglrc);   //Make the new context the current one
	glewExperimental = GL_TRUE; // required for OpenGL 3+
	if (glewInit() != GLEW_OK)
	{
		MessageBox(NULL, _T("Glew Init Failed!"), _T("I AM ERROR"),
				MB_ICONEXCLAMATION | MB_OK);
		exit(-1);
	}
  
	//Next we display the window
	ShowWindow( 
			hWnd, //Handle to the window 
				nCmdShow); //Parameter passed to WinMain
	UpdateWindow(hWnd);
	m_engine.Init();
	//Next we create a message loop to hgandle incoming messages
	MSG msg; //Holds message info https://msdn.microsoft.com/en-us/library/ms644958(v=vs.110).aspx
	while (GetMessage(       //https://msdn.microsoft.com/en-us/library/ms644936(v=vs.110).aspx
			&msg,  //Pointer to the MSG object
			NULL,  //Handle to the window to retireve messages from. NULL means process all messages from thread and window
			0, 0)) //These are used to filter messeage [min, max], ie dont process mouse input. 
	{
		GLfloat bg[3] = {255, 0, 0 };
		glClearBufferfv(GL_COLOR, 0, bg); // clear the color buffer to color bg, which we set to red

		m_engine.Step(1);
      
		SwapBuffers(hdc);  //Swap buffers to new display
		DispatchMessage(&msg);  //Sends message to the window  procedure
	}


	//Release the context
	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(hglrc);
	ReleaseDC(hWnd, hdc);
  
	return (int) msg.wParam;
}
