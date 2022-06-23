package Solitaire.AI;
import Solitaire.AI.GameEngine;

import java.io.IOException;

public class Main {

    GameEngine game;

    GameGUI gui;

    AI SolitaireBot;

    public Main() throws IOException, InterruptedException {
        game = new GameEngine();
        gui = new GameGUI(game);
        SolitaireBot = new AI(game, gui);
    }

    public static void main(String[] args) throws IOException, InterruptedException {
        Main main = new Main();
    }
}


