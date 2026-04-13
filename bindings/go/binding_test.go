package tree_sitter_modelica_test

import (
	"testing"

	tree_sitter "github.com/tree-sitter/go-tree-sitter"
	tree_sitter_modelica "github.com/openmodelica/tree-sitter-modelica/bindings/go"
)

func TestCanLoadGrammar(t *testing.T) {
	language := tree_sitter.NewLanguage(tree_sitter_modelica.Language())
	if language == nil {
		t.Errorf("Error loading Modelica grammar")
	}
}
