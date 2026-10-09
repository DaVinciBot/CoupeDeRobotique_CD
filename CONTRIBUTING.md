# Contribuer au Calcul Déporté

## Branches

| Branche | Rôle | Créée depuis |
|---|---|---|
| `main` | Code testé, prêt pour la compétition | – |
| `dev` | Intégration, branche par défaut | – |
| `feat/nom` | Nouvelle fonctionnalité | `dev` |
| `fix/nom` | Correction de bug | `dev` |
| `perf/nom` | Amélioration de performance | `dev` |
| `test/nom` | Ajout ou modification de tests | `dev` |
| `refactor/nom` | Réorganisation du code sans changement de comportement | `dev` |
| `docs/nom` | Documentation | `dev` |
| `hotfix/nom` | Correction urgente en compétition | `main` |

## Flux de travail

1. Créer la branche depuis `dev`, avec un préfixe (`feat/`, `fix/`, `perf/`, `test/`, `refactor/`, `docs/`).
2. Pousser la branche et ouvrir une PR vers `dev`.
3. Le merge se fait via PR.
4. Quand `dev` est testé et prêt pour la compétition, ouvrir une PR `dev → main`.

## Hotfix

1. Créer `hotfix/nom` depuis `main`.
2. Ouvrir une PR vers `main`.
3. Reporter ensuite le correctif dans `dev` : PR `hotfix/nom → dev`, ou merge de `main` vers `dev`.

## Règles automatiques

- Pas de push direct sur `dev` ni sur `main` : tout passe par une PR.
- Une PR vers `main` doit venir de `dev` ou d'un `hotfix/*`. Sinon le check `main-source` échoue et bloque le merge.
- Un nom de branche sans préfixe reconnu est signalé par un avertissement dans la PR, mais ne bloque pas le merge. Le préfixe reste recommandé pour garder l'historique lisible.
- Pas de force-push ni de suppression sur `dev` et `main`.
