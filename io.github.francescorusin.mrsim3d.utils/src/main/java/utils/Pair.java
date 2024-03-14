package utils;

public record Pair<F, S>(F first, S second) {
    @Override
    public String toString() {
        return String.format("[%s %s]", first.toString(), second.toString());
    }
}
