#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <string>
using namespace std;

atomic<bool> once;
atomic<int> userInputVar;
atomic<int> mainInputVar;

std::mutex mu;
chrono::time_point<chrono::high_resolution_clock> zeroMark;
chrono::time_point<chrono::high_resolution_clock> timer;
chrono::milliseconds timeElapsed;
chrono::milliseconds threshold(650);
bool idleState;


int mygetch() {
	struct termios oldt,
	newt;
	int            ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}





int getMainUserInput(){ return mainInputVar; }

void passToMain(int input){
	mu.lock();

	mainInputVar= input;

	mu.unlock();
}




void userInput(){
	Timer t1("Thread userInput time alive: ");
	while(true){
		
		int userInputVar= mygetch();
		once= false;
		passToMain(userInputVar);
		if(userInputVar == 'x'){
			break;
		}
	}
}

void idle(){
	if(idleState == false  && (timeElapsed.count() > threshold.count())){

		cout << "Idle state" << endl;
		idleState= true;
	}
}


int main(){
	Timer time("Main loop time alive: ");
	std::thread userThread(userInput);
	
	bool mainLoop= true;
	while(mainLoop){
		timer= chrono::high_resolution_clock::now();
		timeElapsed= chrono::duration_cast<chrono::milliseconds>(timer - zeroMark);

		switch(getMainUserInput()){
			case 'x':
				mainLoop= false;
				userThread.join();	
			break;

			case 'w':
				if(once == false){
					cout << "forward" << endl;
					idleState= false;
					zeroMark= chrono::high_resolution_clock::now();
					once= true;
				}
			break;

			case 's':
				if(once == false){
					cout << "reverse" << endl;
					idleState= false;
					zeroMark= chrono::high_resolution_clock::now();
					once= true;
				}
			break;

			default:
			break;
		}
		idle();	
	}

	return 0;

}
