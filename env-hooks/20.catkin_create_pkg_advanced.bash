function _roscomplete_catkin_create_pkg_advanced
{
    local arg opts
    COMPREPLY=()
    arg="${COMP_WORDS[COMP_CWORD]}"
    
    opts=`rospack list | grep -Eo '^[^ ]+' 2> /dev/null`

    if [[ $COMP_CWORD > 1 ]]; then
        COMPREPLY=( $(compgen -W "${opts}" -- ${arg}) )
    fi
    
    return 0
}

complete -F "_roscomplete_catkin_create_pkg_advanced" "catkin_create_pkg_advanced"
