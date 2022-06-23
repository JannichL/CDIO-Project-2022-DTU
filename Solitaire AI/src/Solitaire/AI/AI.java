package Solitaire.AI;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;

public class AI {

    GameEngine game;
    GameGUI gui;
    Card lastProcessedCard;
    private final boolean DEBUG = false;
    int draw = 0;
    int move = 1;
    int games = 0;
    int wins = 0;
    Boolean options = true;
    Boolean foundMove = false;
    Boolean gameover = false;
    Boolean robotEnabled = false;
    SerialCommunication serialCom;

    public AI(GameEngine game, GameGUI gui) throws IOException, InterruptedException {
        this.game = game;
        this.gui = gui;

        if (robotEnabled){
            this.serialCom = new SerialCommunication();
            serialCom.setupSerialCommunication();
            robotSetup();
        }

        game.drawCards();
        Card getPileCard;
        if(robotEnabled) {
             getPileCard = game.getPile.cards.get(game.getPile.cards.size() - 1);
            if (getPileCard.cardSuit == Card.CardSuit.Unknown) {
                String locationOnBoard = game.buddy.get(game.currentCardInGetPile);
                String[] readLocation = locationOnBoard.split(",");
                Card robotReadCard = robotReadCard(1, Integer.parseInt(readLocation[0]), Integer.parseInt(readLocation[0]));
                game.getPile.remove(getPileCard);
                game.getPile.addCard(robotReadCard);
            }
        }

        while(games != 10000){
            draw = 0;
        while (!gameover) {
            if(DEBUG) {
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            gameover = game.checkIfWin();

            switch (move) {
                case 1:
                    //Hvis der kan spilles et es eller en 2'er, så spil dem
                    if (DEBUG) {
                        System.out.println("Checking for ace or 2");
                    }
                    foundMove = aceOrTwo();

                    break;
                case 2:
                    //Hvis der kan rykkes et kort der vender et kort på bordet (Ryk bunke)
                    if (DEBUG) {
                        System.out.println("Checking for freeCardMove");
                    }
                    foundMove = freeCardMove();

                    break;
                case 3:
                    //Hvis der kan rykkes fra den største bunke med face down kort
                    if (DEBUG) {
                        System.out.println("Checking for freeBiggestPile");
                    }
                    foundMove = playCardFromDeck();

                    break;
                case 4:
                    //Kun spil en konge der hjælper med at fjerne kort fra den største bunke medmindre en konge af modsat kulør frigør et face down kort
                    if (DEBUG) {
                        System.out.println("Checking for moveOrPlayKing");
                    }
                    foundMove = moveOrPlayKing();
                    break;
                case 5:
                    //Kun ryk noget af bunke hvis kortet kan rykkes til foundation
                    if(!options) {
                        foundMove = playWhatever();
                    }
                    break;
                case 6:
                    //Kun ryk noget af bunke hvis kortet kan rykkes til foundation
                    if(!options) {
                        foundMove = freeCardToAllowFoundationMove();
                    }
                    break;
                case 7:
                    //Kun spil en konge der hjælper med at fjerne kort fra den største bunke medmindre en konge af modsat kulør frigør et face down kort
                    if(!options) {
                        foundMove = moveFromPilesToFoundation();
                    }
                    break;
                case 8:
                    if(!options) {
                        foundMove = moveFromDeckToFoundation();
                    }
                    break;
                case 9:
                    game.drawCards();
                    if(robotEnabled) {
                        getPileCard = game.getPile.cards.get(game.getPile.cards.size() - 1);
                        if (getPileCard.cardSuit == Card.CardSuit.Unknown) {
                            String locationOnBoard = game.buddy.get(game.currentCardInGetPile);
                            String[] readLocation = locationOnBoard.split("-");
                            game.getPile.remove(getPileCard);
                            game.getPile.addCard(robotReadCard(1, Integer.parseInt(readLocation[0]), Integer.parseInt(readLocation[0])));
                        }
                    }
                    draw++;
                    move = 0;
                    if (draw == game.buddy.size()){
                        options = false;
                    }
                    if (draw == game.buddy.size()*2) {
                        gameover = true;
                    }
                    break;
            }

            //Flip over unblocked cards
            for (int i = 0; i < 7; i++) {
                if (!game.tablePiles.get(i).isEmpty()) {
                    Pile pile = game.tablePiles.get(i);
                    Card card = pile.cards.get(pile.cards.size() - 1);
                    if(card.isReversed){
                        if(robotEnabled) {
                            Card robotReadCard = robotReadCard(1, i, 0);
                            pile.remove(card);
                            pile.addCard(robotReadCard);
                        }
                        card.isReversed = false;
                    }
                }
            }

            if (!foundMove) {
                move++;
            }

            if (foundMove) {
                options = true;
                move = 1;
                foundMove = false;
                draw = 0;
            }

            gui.repaint();

        }

        foundMove = true;
        gameover = false;
        games++;
        if(game.checkIfWin()){
            wins++;
        }

            DecimalFormat df = new DecimalFormat();
            df.setMaximumFractionDigits(1);
            String percentage = df.format((float) wins/games*100);

        System.out.println("Wins: " + wins + " out of " + games + " games - " + percentage + "% Winrate");
        gui.resetGame();
        gui.validate();

    }
    }

    private Boolean aceOrTwo() throws IOException, InterruptedException {
        //Check normal piles and drawpile for ace
        for (int i = 0; i < 8; i++) {

            Pile pile;

            if (i == 7) {
                pile = game.getPile;
            } else {
                pile = game.tablePiles.get(i);
            }
            if (!pile.isEmpty()) {
                Card pileCard = pile.cards.get(pile.cards.size()-1);
                if (pileCard.cardValue == 1) {
                    for(Pile finalPile : game.foundationPiles){
                        if (finalPile.isEmpty()) {
                            if(i==7 && robotEnabled){
                                robotMovePile(pile, finalPile);
                                removeFromDeckRepresentation(pileCard);
                                game.correctIndexingOfDeckPlates();
                                System.out.println(game.buddy);
                            }
                            finalPile.addCard(pileCard);
                            pile.removeCard(pileCard);
                            if (DEBUG) {
                                System.out.println("Match found!");
                            }
                            return true;
                        }
                    }
                } else if (pileCard.cardValue == 2) {
                    //Check final piles first
                    for (int j = 0; j < 4; j++) {
                        Pile finalPile = game.foundationPiles.get(j);
                        if (!finalPile.isEmpty()) {
                            if (finalPile.cards.get(0).cardValue == 1 && finalPile.cards.get(0).cardSuit == pileCard.cardSuit) {
                                if(i==7 && robotEnabled){
                                    robotMovePile(pile, finalPile);
                                    removeFromDeckRepresentation(pileCard);
                                    game.correctIndexingOfDeckPlates();
                                    System.out.println(game.buddy);
                                }
                                finalPile.addCard(pileCard);
                                pile.removeCard(pileCard);
                                if (DEBUG) {
                                    System.out.println("Match found!");
                                }
                                return true;

                            }
                        }
                    }
                    //Check regular piles after
                        for (int j = 0; j < 7; j++) {
                            ArrayList<Card> cards = game.tablePiles.get(j).cards;
                            if (cards.size() > 1) {
                                Card card = cards.get(cards.size() - 1);
                                if ((card.cardValue == 3
                                        && pileCard != lastProcessedCard
                                        && card.cardSuit.isRed != pileCard.cardSuit.isRed)
                                        && (cards.get(cards.size() - 2).isReversed)) {
                                    lastProcessedCard = pileCard;
                                    if(i == 7 && robotEnabled){
                                        robotMovePile(pile, game.tablePiles.get(j));
                                        removeFromDeckRepresentation(pileCard);
                                        game.correctIndexingOfDeckPlates();
                                        System.out.println(game.buddy);
                                    }
                                    game.tablePiles.get(j).addCard(pileCard);
                                    pile.removeCard(pileCard);
                                    if (DEBUG) {
                                        System.out.println("Match found!");
                                    }
                                    return true;
                                }
                            }
                        }
                    }
            }
        }
        if (DEBUG) {
            System.out.println("No match!");
        }
        return false;
    }

    private Boolean freeCardMove() throws IOException, InterruptedException {

        for (Pile pile : game.tablePiles) {
            if (!pile.isEmpty()) {
                        Card card = pile.cards.get(findLastFaceUpCard(pile));
                        for (Pile otherpile : game.tablePiles) {
                            if (!otherpile.isEmpty()) {
                                Card cardCompare = otherpile.cards.get(otherpile.cards.size() - 1);
                                if (pile != otherpile
                                        && card.cardValue == cardCompare.cardValue - 1
                                        && card.cardSuit.isRed != cardCompare.cardSuit.isRed
                                        && card != lastProcessedCard
                                ) {
                                    lastProcessedCard = card;
                                    if(robotEnabled) {
                                        robotMovePile(pile, otherpile);
                                    }
                                    moveFaceUpCards(pile, otherpile);
                                    if (DEBUG) {
                                        System.out.println("Match found!");
                                    }
                                    return true;
                                }
                            }
                       // }
                   // }
                }
            }
        }
        if (DEBUG) {
            System.out.println("No match!");
        }
        return false;
    }

    private boolean playCardFromDeck() throws IOException, InterruptedException {

        if(!game.getPile.isEmpty()) {
            Pile getPile = game.getPile;
            Card getCard = getPile.cards.get(getPile.cards.size() - 1);

            //Check if card from getpile can free a face down card from another pile
            for (Pile pile : game.tablePiles) {
                boolean hasReversed = false;
                Card pileCard = getCard;
                for (Card card : pile.cards) {
                    if (card.isReversed) {
                        hasReversed = true;
                    } else {
                        pileCard = card;
                        break;
                    }
                }

                if ((getCard.cardValue - 1 == pileCard.cardValue) && (getCard.cardSuit.isRed != pileCard.cardSuit.isRed)) {
                    boolean isPossible = checkIfPlayableAndPlay(getPile, getCard);
                    if (isPossible) {
                        for (Pile mergePile : game.tablePiles) {
                            if(!mergePile.isEmpty()) {
                                Card playableCard = mergePile.cards.get(mergePile.cards.size() - 1);
                                if ((playableCard.cardSuit.isRed != pileCard.cardSuit.isRed) && (playableCard.cardValue - 1 == pileCard.cardValue)) {
                                    if(robotEnabled) {
                                        robotMovePile(game.getPile, mergePile);
                                    }
                                    moveFaceUpCards(pile, mergePile);
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }

        return false;
    }
        private boolean moveOrPlayKing() throws IOException, InterruptedException {

            for(Pile pile : game.tablePiles){
                if(!pile.isEmpty()) {
                    Card card = pile.cards.get(findLastFaceUpCard(pile));
                    if (card.cardValue == 13 && card != pile.cards.get(0)) {
                        for (Pile emptypile : game.tablePiles) {
                            if (emptypile.isEmpty()) {
                                if(robotEnabled) {
                                    robotMovePile(pile, emptypile);
                                }
                                moveFaceUpCards(pile, emptypile);
                                return true;
                            }
                        }
                    }
                }
            }

            if(!game.getPile.isEmpty()) {
                //Take top card from getPile
                Card getPileCard = game.getPile.cards.get(game.getPile.cards.size() - 1);

                    if (getPileCard.cardValue == 13) {
                        for (Pile emptypiles : game.tablePiles) {
                            if (emptypiles.isEmpty()) {
                                for(Pile pileWithQueen : game.tablePiles) {
                                    if (!pileWithQueen.isEmpty()) {
                                        Card pileCard = pileWithQueen.cards.get(findLastFaceUpCard(emptypiles));
                                        if (pileCard.cardValue == 12 && getPileCard.cardSuit.isRed != pileCard.cardSuit.isRed) {
                                            if(robotEnabled) {
                                                robotMovePile(game.getPile, emptypiles);
                                                removeFromDeckRepresentation(getPileCard);
                                                game.correctIndexingOfDeckPlates();
                                                System.out.println(game.buddy);
                                            }
                                            emptypiles.addCard(getPileCard);
                                            game.getPile.cards.remove(getPileCard);
                                            return true;
                                        }
                                    }
                                }
                            }
                        }
                    }
            }


        return false;
        }

        private boolean freeCardToAllowFoundationMove() throws IOException, InterruptedException {

            for(Pile finalPile : game.foundationPiles){
                if(!finalPile.isEmpty()) {
                    Card finalPileCard = finalPile.cards.get(finalPile.cards.size() - 1);
                    for (Pile pile : game.tablePiles) {
                        if (!pile.isEmpty()) {
                            for (int i = 0; i < pile.cards.size(); i++) {
                                Card pileCard = pile.cards.get(i);
                                if (!pileCard.isReversed && pileCard.cardValue == finalPileCard.cardValue + 1 && pileCard.cardSuit == finalPileCard.cardSuit && pile.cards.size() != i+1){
                                    Card cardToMove = pile.cards.get(i+1);
                                    for(Pile toPile : game.tablePiles){
                                        if(!toPile.isEmpty()) {
                                            Card toCard = toPile.cards.get(toPile.cards.size() - 1);
                                            if(toCard.cardValue == cardToMove.cardValue + 1 && toCard.cardSuit.isRed != cardToMove.cardSuit.isRed){
                                                if(robotEnabled){
                                                    robotMovePile(pile, toPile);
                                                }
                                                moveCardsFromIndex(pile, toPile, i+1);
                                                finalPile.addCard(pileCard);
                                                pile.removeCard(pileCard);
                                                return true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

        return false;
        }

    private boolean playWhatever() throws IOException, InterruptedException {

        if(!game.getPile.isEmpty()) {
            Card getPileCard = game.getPile.cards.get(game.getPile.cards.size() - 1);

            if(getPileCard.cardValue == 13){
                for(Pile pile : game.tablePiles){
                    if(pile.isEmpty()){
                        if(robotEnabled) {
                            robotMovePile(game.getPile, pile);
                            removeFromDeckRepresentation(getPileCard);
                            game.correctIndexingOfDeckPlates();
                            System.out.println(game.buddy);
                        }
                        pile.addCard(getPileCard);
                        game.getPile.removeCard(getPileCard);
                        return true;
                    }
                }
            }

            for (Pile pile : game.tablePiles) {
                if(!pile.isEmpty()) {
                    Card pileCard = pile.cards.get(pile.cards.size() - 1);
                    if ((pileCard.cardValue - 1 == getPileCard.cardValue) && (pileCard.cardSuit.isRed != getPileCard.cardSuit.isRed)) {
                        if(robotEnabled) {
                            robotMovePile(game.getPile, pile);
                            removeFromDeckRepresentation(getPileCard);
                            game.correctIndexingOfDeckPlates();
                            System.out.println(game.buddy);
                        }
                        pile.addCard(getPileCard);
                        game.getPile.removeCard(getPileCard);
                        return true;
                    }
                }
            }
        }

        return false;
    }

    private boolean moveFromPilesToFoundation() throws IOException, InterruptedException {

        for(Pile pile : game.tablePiles){
            if(!pile.isEmpty()){
                Card pileCard = pile.cards.get(pile.cards.size()-1);
                for(Pile finalPiles : game.foundationPiles){
                    if(!finalPiles.isEmpty()){
                        Card finalPileCard = finalPiles.cards.get(finalPiles.cards.size()-1);
                        if(finalPileCard.cardValue == pileCard.cardValue-1 && finalPileCard.cardSuit == pileCard.cardSuit){
                            if(robotEnabled) {
                                robotMovePile(pile, finalPiles);
                            }
                            finalPiles.addCard(pileCard);
                            pile.removeCard(pileCard);
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }

    private boolean moveFromDeckToFoundation(){

        if(!game.getPile.isEmpty()){
            Card getPileCard = game.getPile.cards.get(game.getPile.cards.size()-1);

            for(Pile finalPiles : game.foundationPiles){
                if(!finalPiles.isEmpty()){
                    Card finalCard = finalPiles.cards.get(finalPiles.cards.size()-1);
                    if(getPileCard.cardValue-1 == finalCard.cardValue && getPileCard.cardSuit == finalCard.cardSuit){
                        if(robotEnabled) {
                            removeFromDeckRepresentation(getPileCard);
                            game.correctIndexingOfDeckPlates();
                            System.out.println(game.buddy);
                        }
                        finalPiles.addCard(getPileCard);
                        game.getPile.removeCard(getPileCard);
                        return true;
                    }
                }
            }
        }
        return false;
    }

        private void moveCardsFromIndex(Pile fromPile, Pile toPile, int index) throws IOException, InterruptedException {
            int size = fromPile.cards.size();
            for(int i = 0; i < size - index; i++){
                if(robotEnabled) {
                    robotMovePile(fromPile, toPile);
                }
                toPile.addCard(fromPile.cards.get(index));
                fromPile.removeCard(fromPile.cards.get(index));
            }
        }
        private void moveFaceUpCards(Pile fromPile, Pile toPile) throws IOException, InterruptedException {

        int index = findLastFaceUpCard(fromPile);
            int size = fromPile.cards.size();
            for(int i = 0; i < size - index; i++){
                if(robotEnabled) {
                    robotMovePile(fromPile, toPile);
                }
                toPile.addCard(fromPile.cards.get(index));
                fromPile.removeCard(fromPile.cards.get(index));
            }
        }

        private int findLastFaceUpCard(Pile pile){

            if(!pile.isEmpty()){
                for(int i = 0; i < pile.cards.size(); i++) {
                    if(!pile.cards.get(i).isReversed){
                        return i;
                    }
                }
            }
            return 0;
        }

        private boolean checkIfPlayableAndPlay(Pile fromPile, Card card){

            for(Pile pile : game.tablePiles){
                if(!pile.isEmpty()) {
                    Card pileCard = pile.cards.get(pile.cards.size() - 1);
                    if ((card.cardSuit.isRed != pileCard.cardSuit.isRed) && card.cardValue == pileCard.cardValue - 1) {
                        pile.addCard(card);
                        fromPile.removeCard(card);
                        if(robotEnabled) {
                            if (fromPile.pileType == Pile.PileType.Get) {
                                removeFromDeckRepresentation(card);
                                game.correctIndexingOfDeckPlates();
                                System.out.println(game.buddy);
                            }
                        }
                        return true;
                    }
                }
            }
            return false;
        }

        private void removeFromDeckRepresentation(Card cardToRemove){
            for (int i = 0; i < 6; i++) {
                Pile pile = game.deckPilesPlateRepresentation.get(i);
                for (int k = 0; k < pile.cards.size(); k++) {
                    if(pile.cards.get(k) == cardToRemove){
                        pile.removeCard(cardToRemove);
                    }
                }
            }
        }

        private void robotSetup() throws IOException, InterruptedException {
            for(int i = 0; i < 7; i++){

                Pile pile = game.tablePiles.get(i);

                pile.addCard(robotReadCard(1,i+1,0));
                pile.removeCard(pile.cards.get(pile.cards.size()-1));

                gui.repaint();
                gui.validate();

                Thread.sleep(1000);

            }
        }

        private void robotMovePile(Pile fromPile, Pile toPile) throws IOException, InterruptedException {

            if (robotEnabled) {
                String command = "";
                String commandType = "";
                if (fromPile.pileType != Pile.PileType.Get) {
                    commandType = "moveCardTable";
                    if (fromPile.pileType == Pile.PileType.Normal) {
                        for (int i = 0; i < 7; i++) {
                            if (fromPile == game.tablePiles.get(i)) {
                                command = 1 + "," + String.valueOf(i) + ",";
                                break;
                            }
                        }
                    }

                    if (toPile.pileType == Pile.PileType.Final) {
                        for (int i = 0; i < 4; i++) {
                            if (toPile == game.foundationPiles.get(i)) {
                                command += 2 + "," + String.valueOf(i);
                                break;
                            }
                        }
                    } else if (toPile.pileType == Pile.PileType.Normal) {
                        for (int i = 0; i < 7; i++) {
                            if (toPile == game.tablePiles.get(i)) {
                                command += 1 + "," + String.valueOf(i);
                                break;
                            }
                        }
                    }
                    System.out.println("CommandType: " + commandType + " Command: " + command);
                } else {
                    commandType = "moveCardDeck";
                    command = 0 + "," + game.buddy.get(game.currentCardInGetPile);
                }
                serialCom.sendCommandToArduino(commandType, command);
            }
        }


        private Card robotReadCard(int plate, int spot, int index) throws IOException, InterruptedException {

            String command = "";

            command = plate + "," + spot + "," + index;

            serialCom.sendCommandToArduino("readCard", command);

            Boolean loop = false;

            StringBuilder readString = new StringBuilder();
            char[] readStringArr = readString.toString().toCharArray();
            char[] compareStringArr = "ready".toCharArray();

            while(!loop) {
                //Read from txt
                try {
                    FileReader reader = new FileReader("/Users/jannich/Downloads/OpenCV-Playing-Card-Detector-master/ranksuit.txt");
                    int character;

                    readString = new StringBuilder();

                    while ((character = reader.read()) != -1) {
                        readString.append((char) character);
                    }
                    reader.close();

                    System.out.println("I read this: " + readString);

                } catch (IOException e) {
                    e.printStackTrace();
                }

                for(int i = 0; i < 5; i++){
                    if(readStringArr[i] != compareStringArr[i]){
                        loop = false;
                        break;
                    } else {
                        loop = true;
                    }
                }

            }

            readStringArr = readString.toString().toCharArray();

            Card card = new Card(0, Card.CardSuit.Unknown);

            switch (readStringArr[1]){
                case 'S':
                    card.cardSuit = Card.CardSuit.Spades;
                    break;
                case 'H':
                    card.cardSuit = Card.CardSuit.Hearts;
                    break;
                case 'D':
                    card.cardSuit = Card.CardSuit.Diamonds;
                    break;
                case 'C':
                    card.cardSuit = Card.CardSuit.Clubs;
                    break;
            }

            card.cardValue = Integer.parseInt(String.valueOf(readStringArr[0]));

            try {
                FileWriter writer = new FileWriter("/Users/jannich/Downloads/OpenCV-Playing-Card-Detector-master/ranksuit.txt");

                writer.write("wait");
                writer.close();

            } catch (IOException e) {
                e.printStackTrace();
            }

            return new Card(card.cardValue, card.cardSuit); //Value of card;
        }
    }


