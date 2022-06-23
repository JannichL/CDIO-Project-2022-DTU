package Solitaire.AI;
import Solitaire.AI.Card.CardSuit;

import java.awt.Component;
import java.awt.Dimension;
import java.util.ArrayList;
import javax.swing.JLayeredPane;

public class Pile extends JLayeredPane {

    Card base;
    ArrayList<Card> cards;
    int offset = 15;
    CardSuit cardSuit;
    int width;
    Pile pileParent;
    PileType pileType;

    enum PileType {
        Normal, Draw, Get, Final
    };

    public Pile(int width) {
        cards = new ArrayList<Card>();
        this.width = width;

        base = new Card(100, CardSuit.Spades);
        add(base, 1, 0);

        pileType = PileType.Normal;
    }

    public void addCard(Card card) {
        card.setLocation(0, offset * cards.size());
        cards.add(card);

        this.add(card, 1, 0);
        updateSize();
    }

    public void removeCard(Card card) {
        cards.remove(card);
        this.remove(card);
        updateSize();
    }

    public Card drawCard() {
        Card c = cards.get(0);
        removeCard(c);
        return c;
    }

    public void updateSize() {
        int height = base.getSize().height;

        if (!cards.isEmpty()) {
            height += offset * (cards.size() - 1);
        }

        this.setPreferredSize(new Dimension(width, height));
        this.setSize(width, height);
    }

    public void setOffset(int offset) {
        this.offset = offset;
        updateSize();
    }

    public boolean isEmpty() {
        return cards.size() == 0;
    }

    public boolean isOptimizedDrawingEnabled() {
        return false;
    }

    @Override
    public String toString() {
        String result = "";
        result += base.saveAsString() + "-";
        for (Card card : cards) {
            result += card.saveAsString() + "-";
        }

        return result;
    }

    @Override
    public Component.BaselineResizeBehavior getBaselineResizeBehavior() {
        return Component.BaselineResizeBehavior.CONSTANT_ASCENT;
    }

    @Override
    public int getBaseline(int width, int height) {
        return 0;
    }
}
