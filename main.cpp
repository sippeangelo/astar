#include <iostream>
#include "DV1419Map.h"
#include "ScenarioLoader.h"
#include "AStar.hpp"
#include "timer.h"
#include <SFML/Graphics.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <string>
#include <sstream>

void graphical()
{
	std::cout << "Loading map..." << std::endl;
	DV1419Map map = DV1419Map("maps/brc505d.map");
	std::cout << "Map Width: " << map.getWidth() << std::endl;
	std::cout << "Map Height: " << map.getHeight() << std::endl;
	// Load the scenario
	ScenarioLoader scenario = ScenarioLoader("maps/brc505d.map.scen");
	std::cout << "Loaded scenario " << scenario.GetScenarioName() << std::endl;

	AStar aStar = AStar(&map, *AStar::Heuristics::Diagonal);
	/*aStar.Prepare(
		Coordinate(100, 160), 
		Coordinate(76, 24)
		);*/

	sf::RenderWindow window(sf::VideoMode(800, 600), "A*");
	sf::RectangleShape mapBrush = sf::RectangleShape(sf::Vector2f(2, 2));
	mapBrush.setFillColor(sf::Color::Blue);
	sf::RectangleShape openListBrush = sf::RectangleShape(sf::Vector2f(2, 2));
	openListBrush.setFillColor(sf::Color::Cyan);
	sf::RectangleShape closedListBrush = sf::RectangleShape(sf::Vector2f(2, 2));
	closedListBrush.setFillColor(sf::Color(50, 50, 50));
	sf::RectangleShape pathBrush = sf::RectangleShape(sf::Vector2f(2, 2));
	pathBrush.setFillColor(sf::Color::Green);

	// Pre-render the map
	sf::RenderTexture mapTexture;
	mapTexture.create(map.getWidth() * 2, map.getHeight() * 2);
	mapTexture.clear();
	for (int x = 0; x < map.getWidth(); x++)
	{
		for (int y = 0; y < map.getHeight(); y++)
		{
			if (!map.isWalkable(x, y))
			{
				mapBrush.setPosition(x * 2, y * 2);
				mapTexture.draw(mapBrush);
			}
		}
	}
	mapTexture.display();

	sf::Clock clock;
	sf::Time pressTime;
	sf::Time lastExperimentChange;

	Timer timer;
	int totalTime = 0;
	bool timeDisplayed = false;

	int currentExperiment = 0;
	Experiment experiment = scenario.GetNthExperiment(currentExperiment);
						aStar.Prepare(
							Coordinate(experiment.GetStartX(), experiment.GetStartY()), 
							Coordinate(experiment.GetGoalX(), experiment.GetGoalY())
							);
						std::cout << "Experiment Map Width: " << experiment.GetXScale() << std::endl;
						std::cout << "Experiment Map Height: " << experiment.GetYScale() << std::endl;


	//std::vector<Coordinate>* path = nullptr;
	AStar::Node* foundGoal = nullptr;
	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}

		bool leftPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::Left);
		bool rightPressed = sf::Keyboard::isKeyPressed(sf::Keyboard::Right);
		if (leftPressed || rightPressed)
		{
			if ((clock.getElapsedTime() - pressTime).asMilliseconds() > 500 || pressTime == sf::Time::Zero)
			{
				if ((clock.getElapsedTime() - lastExperimentChange).asMilliseconds() > 20)
				{
					currentExperiment += (leftPressed && !rightPressed) ? -1 : 1;
					if (currentExperiment >= 0 && currentExperiment < scenario.GetNumExperiments())
					{
						std::cout << "Experiment " << currentExperiment << std::endl;
						experiment = scenario.GetNthExperiment(currentExperiment);
						aStar.Prepare(
							Coordinate(experiment.GetStartX(), experiment.GetStartY()), 
							Coordinate(experiment.GetGoalX(), experiment.GetGoalY())
							);
						foundGoal = nullptr;
						totalTime = 0;
						timeDisplayed = false;
						lastExperimentChange = clock.getElapsedTime();
					}
					else
					{
						currentExperiment -= (leftPressed && !rightPressed) ? -1 : 1;
					}

					lastExperimentChange = clock.getElapsedTime();
				}
				
			}

			if (pressTime == sf::Time::Zero)
			{
				pressTime = clock.getElapsedTime();
			}
		}
		else
		{
			pressTime = sf::Time::Zero;
		}
		
		for (int i = 0; i < 25; i++)
		{
			if (foundGoal != nullptr)
			{
				if (!timeDisplayed)
				{
					std::vector<Coordinate>* path = aStar.ReconstructPath(foundGoal);
					double length = 0;
					if (path->size() != 0)
						length = map.getPathLength(*path);
					std::cout << "Time: " << totalTime / 1000.0f << " ms" << std::endl;
					std::cout << "Length: " << length << std::endl;
					std::cout << "Optimal Length: " << experiment.GetDistance() << std::endl;
					timeDisplayed = true;
				}
				break;
			}

			timer.start();
			foundGoal = aStar.Update();
			timer.stamp();
			totalTime += timer.getTimePassed();
		}

		window.clear();

		window.draw(sf::Sprite(mapTexture.getTexture()));

		/*for (auto it = aStar.m_ClosedList.begin(); it != aStar.m_ClosedList.end(); it++)
		{
			closedListBrush.setPosition((*it)->X * 2, (*it)->Y * 2);
			window.draw(closedListBrush);
		}

		for (auto it = aStar.m_OpenList.begin(); it != aStar.m_OpenList.end(); it++)
		{
			openListBrush.setPosition((*it)->X * 2, (*it)->Y * 2);
			window.draw(openListBrush);
		}*/

		for (int x = 0; x < map.getWidth(); x++)
		{
			for (int y = 0; y < map.getHeight(); y++)
			{
				int i = y * map.getWidth() + x;
				AStar::Node* node = aStar.m_Nodes[i];
				if (node != nullptr)
				{
					if (node->Open)
					{
						openListBrush.setPosition(x * 2, y * 2);
						window.draw(openListBrush);
					}
					if (node->Closed)
					{
						closedListBrush.setPosition(x * 2, y * 2);
						window.draw(closedListBrush);
					}
				}
			}
		}

		for (AStar::Node* n = aStar.m_CurrentNode; n != nullptr; n = n->Parent)
		{
			pathBrush.setPosition(n->X * 2, n->Y * 2);
			window.draw(pathBrush);
		}

		window.display();
	}
}

void tests()
{
	std::cout << "Loading map..." << std::endl;
	DV1419Map map = DV1419Map("maps/den005d.map");
	std::cout << "Map Width: " << map.getWidth() << std::endl;
	std::cout << "Map Height: " << map.getHeight() << std::endl;

	Timer timer = Timer();

	// Load the scenario
	ScenarioLoader scenario = ScenarioLoader("maps/den005d.map.scen");
	std::cout << "Loaded scenario " << scenario.GetScenarioName() << std::endl;

	
	AStar aStar = AStar(&map, *AStar::Heuristics::Euclidean);

	sf::RenderWindow window(sf::VideoMode(800, 600), "A*");


	// Run all experiments
	int failures = 0;
	int total = 0;
	for (int experimentNumber = 0; experimentNumber < scenario.GetNumExperiments(); experimentNumber++)
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}


		Experiment experiment = scenario.GetNthExperiment(experimentNumber);
		std::cout << "Loaded experiment #" << experimentNumber << std::endl;
		std::cout << "\tStart: (" << experiment.GetStartX() << ", " << experiment.GetStartY() << ")" << std::endl;
		std::cout << "\tGoal: (" << experiment.GetGoalX() << ", " << experiment.GetGoalY() << ")" << std::endl;
		std::cout << "\tOptimal Length: " << experiment.GetDistance() << std::endl;
		std::cout << "Finding path..." << std::endl;
		timer.start();
		std::vector<Coordinate>* path = aStar.Path(
			Coordinate(experiment.GetStartX(), experiment.GetStartY()), 
			Coordinate(experiment.GetGoalX(), experiment.GetGoalY())
		);
		timer.stamp();
		double pathLength = map.getPathLength(*path);
		std::cout << "Path Length: " << pathLength;
		if (abs(pathLength - experiment.GetDistance()) > 1)
		{
			std::cout << " (FAILED)";
			failures++;
		}
		else
			std::cout << " (Passed)";
		std::cout << std::endl;

		std::cout << "Time: " << (float)(timer.getTimePassed() / 1000.0f) << " ms" << std::endl;

		total++;

		window.clear();

		for (int x = 0; x < map.getWidth(); x++)
			for (int y = 0; y < map.getHeight(); y++)
			{
				if (!map.isWalkable(x, y))
				{
					sf::RectangleShape square = sf::RectangleShape(sf::Vector2f(2, 2));
					square.setPosition(x * 2, y * 2);
					square.setFillColor(sf::Color::Blue);
					window.draw(square);
				}
			}

		for (auto it = path->begin(); it != path->end(); it++)
		{
			sf::RectangleShape square = sf::RectangleShape(sf::Vector2f(2, 2));
			square.setPosition((*it).X * 2, (*it).Y * 2);
			if (it == path->end()-1)
				square.setFillColor(sf::Color::Red);
			else
				square.setFillColor(sf::Color::Green);
			window.draw(square);
		}

		window.display();

		//if (experimentNumber > 400)
		//	system("pause");
	}

	std::cout << std::endl;
	std::cout << "Failure rate: " << failures << " / " << total << " (" <<  ((float)failures/(float)total) * 100.0f << "%)" << std::endl;
	
	std::cout << std::endl;
	

	/*std::cout << "Path: " << std::endl;
	for (auto it = path.begin(); it != path.end(); it++)
		std::cout << (*it).toString() << std::endl;

	std::cout << "Number of nodes: " << path.size() << std::endl;

	std::cout << "Length: " << map.getPathLength(path) << std::endl;*/

}

int main(int argc, char* argv[])
{
	//graphical();
	//return 0;

	if (argc > 1)
	{
		std::string mapFile = argv[1];
		std::ostringstream scenarioFile;
		scenarioFile << mapFile << ".scen";

		// Load the map
		DV1419Map map = DV1419Map(mapFile.c_str());
		// Load the pathfinder
		AStar aStar = AStar(&map, *AStar::Heuristics::Diagonal);
		// Load the scenario
		ScenarioLoader scenario = ScenarioLoader(scenarioFile.str().c_str());

		int startExperiment = 0;
		int endExperiment = scenario.GetNumExperiments() - 1;

		// Run a specific experiment if a second argument is supplied
		if (argc > 2)
		{
			istringstream(argv[2]) >> startExperiment;
			endExperiment = startExperiment;
		}
		// Run a range of experiments if a third argument is supplied
		if (argc > 3)
		{
			istringstream(argv[3]) >> endExperiment;
			if (endExperiment > scenario.GetNumExperiments() - 1)
				endExperiment = scenario.GetNumExperiments() - 1;
		}

		std::cout << "Running experiment";
		if (startExperiment == endExperiment)
			std::cout << " " << startExperiment << std::endl;
		else
			std::cout << "s " << startExperiment << " through " << endExperiment << std::endl;

		// Run the experiment(s)
		Timer timer;
		unsigned int totalTime = 0;
		int failCount = 0;
		int nodesExpanded = 0;
		for (int i = startExperiment; i <= endExperiment; i++)
		{
			Experiment experiment = scenario.GetNthExperiment(i);
			Coordinate start = Coordinate(experiment.GetStartX(), experiment.GetStartY());
			Coordinate goal = Coordinate(experiment.GetGoalX(), experiment.GetGoalY());

			std::cout << "Start: " << start.toString() << std::endl;
			std::cout << "Goal: " << goal.toString() << std::endl;

			aStar.Prepare(start, goal);
			AStar::Node* foundGoal = nullptr;
			timer.start();
			while (foundGoal == nullptr)
				foundGoal = aStar.Update();
			timer.stamp();

			for (int i = 0; i < map.getWidth() * map.getHeight(); i++)
			{
				if (aStar.m_Nodes[i] != nullptr)
					nodesExpanded++;
			}

			std::vector<Coordinate>* path = aStar.ReconstructPath(foundGoal);

			totalTime += timer.getTimePassed();
			
			double pathLength = 0;
			if (path->size() != 0)
				pathLength = map.getPathLength(*path);
			
			double optimalLength = experiment.GetDistance();

			bool failed = abs(pathLength - optimalLength) >= 1;
			if (failed)
				failCount++;

			std::cout << "#" << i << std::endl;
			std::cout << "Time: " << timer.getTimePassed() / 1000.0f << " ms" << std::endl;
			std::cout << "Length: " << pathLength << (failed ? " != " : " == ") << optimalLength << std::endl;

			if (failed)
				std::cout << "------- FAILED -------" << std::endl;

			delete path;
		}

		std::cout << "Total time: " << totalTime / 1000.0f << " ms" << std::endl;
		std::cout << "Nodes expanded: " << nodesExpanded << std::endl;
		std::cout << "Average time: " << (float)(totalTime / 1000.0f) / (float)(endExperiment - startExperiment) << " ms" << std::endl;
		std::cout << "Failure rate: " << failCount << " / " << endExperiment - startExperiment << " (" <<  ((float)failCount/(float)(endExperiment - startExperiment)) * 100.0f << "%)" << std::endl;
	}
	else
	{
		std::cerr << "No map file specified!" << std::endl;
	}


	return 0;
}