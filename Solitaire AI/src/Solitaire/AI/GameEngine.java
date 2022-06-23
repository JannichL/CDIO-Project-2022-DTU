package Solitaire.AI;

import java.util.ArrayList;

import Solitaire.AI.Pile.PileType;
public class GameEngine {

    ArrayList<Pile> tablePiles;
    ArrayList<Pile> foundationPiles;
    Pile deckPile, getPile;
    ArrayList<Pile> allPiles;
    ArrayList<String> buddy;
    ArrayList<Pile> deckPilesPlateRepresentation;
    ArrayList<Pile> deckOrderRepresentation;

    public final int pileNumber = 7;
    int talonCardvar;
    int talonCardCounter;
    int currentCardInGetPile;
    Boolean DEBUG = false;

    public Deck deck;

    public GameEngine() {
        generateCards();
    }

    public void generateCards() {

        deck = new Deck();
        deck.shuffle();

        deckPile = new Pile(120);
        deckPile.setOffset(0);

        getPile = new Pile(180);
        getPile.setOffset(0);

        foundationPiles = new ArrayList<Pile>();
        tablePiles = new ArrayList<Pile>();

        allPiles = new ArrayList<Pile>();
        allPiles.add(deckPile);
        allPiles.add(getPile);

        buddy = new ArrayList<String>();

        deckPilesPlateRepresentation = new ArrayList<Pile>();
        deckOrderRepresentation = new ArrayList<Pile>();

        talonCardvar = 0;
        talonCardCounter = 0;
        currentCardInGetPile = 0;

    }

    public void setupGame(){
        deckPile.pileType = PileType.Draw;
        getPile.pileType = PileType.Get;

        for (int i = 1; i <= pileNumber; ++i) {
            Pile pile = new Pile(120);
            for (int j = 1; j <= i; ++j) {
                Card card = deck.drawCard();
                pile.addCard(card);

                if (j != i) {
                    card.hide();
                } else {
                    card.show();
                }
            }

            tablePiles.add(pile);
            allPiles.add(pile);
        }

        for(int i = 0; i < 4; i++) {
                Pile pile = new Pile(100);
                pile.setOffset(0);
                pile.pileType = PileType.Final;
                foundationPiles.add(pile);
                allPiles.add(pile);
        }

        for (int i = 0; i < 6; i++) {
            Pile pile = new Pile(120);
            deckPilesPlateRepresentation.add(pile);
            if(DEBUG){
                System.out.println("Index: " + i);
            }
            for (int j = 0; j < 4; j++) {
                Card card = deck.drawCard();
                card.hide();
                deckPile.addCard(card);
                pile.addCard(card);

                if(DEBUG) {
                    System.out.println(deckPilesPlateRepresentation.get(i).cards.get(j));
                }
            }
        }

        correctIndexingOfDeckPlates();

        if(DEBUG) {
            System.out.println(buddy);
        }
    }

    public void drawCards() {

        if (deckPile.isEmpty()) {
            while (!getPile.isEmpty()) {
                deckPile.addCard(getPile.cards.get(getPile.cards.size() - 1));
                getPile.removeCard(getPile.cards.get(getPile.cards.size() - 1));
                currentCardInGetPile = 0;
            }

            //Hide all cards in deck
            for (int j = 0; j < deckPile.cards.size(); j++) {
                deckPile.cards.get(j).hide();
            }
        }

        for (int i = 0; i < 3; i++) {
            if (!deckPile.isEmpty()) {
                Card drewCard = deckPile.drawCard();
                drewCard.isReversed = false;
                getPile.addCard(drewCard);
                if(i == 2) currentCardInGetPile++;
            } else {
                currentCardInGetPile++;
                break;
            }
        }
    }

    public boolean checkIfWin() {
        boolean flag=true;
        for (Pile pile : foundationPiles) {
            if (pile.cards.size() != 13) {
                return false;
            }
        }
        return true;
    }

    public void correctIndexingOfDeckPlates() {

        while (!buddy.isEmpty()) {
            buddy.remove(0);
        }

        for (int i = 0; i < 6; i++) {
            for (int k = 0; k < deckPilesPlateRepresentation.get(i).cards.size(); k++) {

                if (talonCardvar + 3 == talonCardCounter || (deckPilesPlateRepresentation.get(5).cards.size() - 1 == k && i == 5)) {
                    talonCardvar = talonCardCounter;

                    buddy.add(i + "," + k);
                }
                talonCardCounter++;
            }
        }
        talonCardvar = 0;
        talonCardCounter = 0;
    }
}
